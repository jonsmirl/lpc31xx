/*
 * ALSA SoC codec Out Audio Layer for spear processors
 *
 * Copyright (C) 2012 ST Microelectronics
 * Vipin Kumar <vipin.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <sound/dmaengine_pcm.h>
#include <sound/soc.h>
#include "sunxi-codec.h"

struct sunxi_params {
	u32 rate;
	u32 core_freq;
	u32 mute;
};

struct sunxi_dev {
	struct clk *clk;
	struct sunxi_codec_dma_params dma_params;
	struct sunxi_params saved_params;
	u32 running;
	void __iomem *io_base;
	struct snd_dmaengine_dai_dma_data dma_params_tx;
	struct snd_dmaengine_pcm_config config;
};


static void sunxi_configure(struct sunxi_dev *host)
{
#ifdef JDS
	writel(sunxi_RESET, host->io_base + sunxi_SOFT_RST);
	mdelay(1);
	writel(readl(host->io_base + sunxi_SOFT_RST) & ~sunxi_RESET,
			host->io_base + sunxi_SOFT_RST);

	writel(sunxi_FDMA_TRIG_16 | sunxi_MEMFMT_16_16 |
			sunxi_VALID_HW | sunxi_USER_HW |
			sunxi_CHNLSTA_HW | sunxi_PARITY_HW,
			host->io_base + sunxi_CFG);

	writel(0x7F, host->io_base + sunxi_INT_STA_CLR);
	writel(0x7F, host->io_base + sunxi_INT_EN_CLR);
#endif 
}

static int sunxi_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *cpu_dai)
{
	struct sunxi_dev *host = snd_soc_dai_get_drvdata(cpu_dai);
	int ret;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	ret = clk_enable(host->clk);
	if (ret)
		return ret;

	host->running = true;
	sunxi_configure(host);

	return 0;
}

static void sunxi_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct sunxi_dev *host = snd_soc_dai_get_drvdata(dai);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return;

	clk_disable(host->clk);
	host->running = false;
}

static void sunxi_clock(struct sunxi_dev *host, u32 core_freq,
		u32 rate)
{
#ifdef JDS
	u32 divider, ctrl;
	clk_set_rate(host->clk, core_freq);
	divider = DIV_ROUND_CLOSEST(clk_get_rate(host->clk), (rate * 128));

	ctrl = readl(host->io_base + sunxi_CTRL);
	ctrl &= ~codec_DIVIDER_MASK;
	ctrl |= (divider << codec_DIVIDER_SHIFT) & codec_DIVIDER_MASK;
	writel(ctrl, host->io_base + sunxi_CTRL);
#endif
}

static int sunxi_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct sunxi_dev *host = snd_soc_dai_get_drvdata(dai);
	u32 rate, core_freq;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	rate = params_rate(params);

	switch (rate) {
	case 8000:
	case 16000:
	case 32000:
	case 64000:
		/*
		 * The clock is multiplied by 10 to bring it to feasible range
		 * of frequencies for sscg
		 */
		core_freq = 64000 * 128 * 10;	/* 81.92 MHz */
		break;
	case 5512:
	case 11025:
	case 22050:
	case 44100:
	case 88200:
	case 176400:
		core_freq = 176400 * 128;	/* 22.5792 MHz */
		break;
	case 48000:
	case 96000:
	case 192000:
	default:
		core_freq = 192000 * 128;	/* 24.576 MHz */
		break;
	}

	sunxi_clock(host, core_freq, rate);
	host->saved_params.core_freq = core_freq;
	host->saved_params.rate = rate;

	return 0;
}

static int sunxi_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
#ifdef JDS
	struct sunxi_dev *host = snd_soc_dai_get_drvdata(dai);
	u32 ctrl;
#endif
	int ret = 0;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
#ifdef JDS		
			ctrl = readl(host->io_base + sunxi_CTRL);
			ctrl &= ~codec_OPMODE_MASK;
			if (!host->saved_params.mute)
				ctrl |= codec_OPMODE_AUD_DATA |
					codec_STATE_NORMAL;
			else
				ctrl |= codec_OPMODE_MUTE_PCM;
			writel(ctrl, host->io_base + sunxi_CTRL);
#endif
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
#ifdef JDS		
		ctrl = readl(host->io_base + sunxi_CTRL);
		ctrl &= ~codec_OPMODE_MASK;
		ctrl |= codec_OPMODE_OFF;
		writel(ctrl, host->io_base + sunxi_CTRL);
#endif
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int sunxi_digital_mute(struct snd_soc_dai *dai, int mute)
{
#ifdef JDS
	struct sunxi_dev *host = snd_soc_dai_get_drvdata(dai);
	u32 val;

	host->saved_params.mute = mute;
	val = readl(host->io_base + sunxi_CTRL);
	val &= ~codec_OPMODE_MASK;

	if (mute)
		val |= codec_OPMODE_MUTE_PCM;
	else {
		if (host->running)
			val |= codec_OPMODE_AUD_DATA | codec_STATE_NORMAL;
		else
			val |= codec_OPMODE_OFF;
	}

	writel(val, host->io_base + sunxi_CTRL);
#endif
	return 0;
}

static int codec_mute_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct sunxi_dev *host = snd_soc_dai_get_drvdata(cpu_dai);

	ucontrol->value.integer.value[0] = host->saved_params.mute;
	return 0;
}

static int codec_mute_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct sunxi_dev *host = snd_soc_dai_get_drvdata(cpu_dai);

	if (host->saved_params.mute == ucontrol->value.integer.value[0])
		return 0;

	sunxi_digital_mute(cpu_dai, ucontrol->value.integer.value[0]);

	return 1;
}
static const struct snd_kcontrol_new sunxi_controls[] = {
	SOC_SINGLE_BOOL_EXT("IEC958 Playback Switch", 0,
			codec_mute_get, codec_mute_put),
};

static int sunxi_soc_dai_probe(struct snd_soc_dai *dai)
{
	struct sunxi_dev *host = snd_soc_dai_get_drvdata(dai);

	host->dma_params_tx.filter_data = &host->dma_params;
	dai->playback_dma_data = &host->dma_params_tx;

	return snd_soc_add_dai_controls(dai, sunxi_controls,
				ARRAY_SIZE(sunxi_controls));
}

static const struct snd_soc_dai_ops sunxi_dai_ops = {
	.digital_mute	= sunxi_digital_mute,
	.startup	= sunxi_startup,
	.shutdown	= sunxi_shutdown,
	.trigger	= sunxi_trigger,
	.hw_params	= sunxi_hw_params,
};

static struct snd_soc_dai_driver sunxi_dai = {
	.name		= "cat",
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 | \
				 SNDRV_PCM_RATE_192000),
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.probe = sunxi_soc_dai_probe,
	.ops = &sunxi_dai_ops,
};

static const struct snd_soc_component_driver sunxi_component = {
	.name		= "horse",
};

#define STUB_RATES	SNDRV_PCM_RATE_8000_96000
#define STUB_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE)

static const struct snd_soc_dapm_widget dit_widgets[] = {
	SND_SOC_DAPM_OUTPUT("spdif-out"),
};

static const struct snd_soc_dapm_route dit_routes[] = {
	{ "spdif-out", NULL, "Playback" },
};

static struct snd_soc_codec_driver soc_codec_sunxi_codec = {
	.dapm_widgets = dit_widgets,
	.num_dapm_widgets = ARRAY_SIZE(dit_widgets),
	.dapm_routes = dit_routes,
	.num_dapm_routes = ARRAY_SIZE(dit_routes),
};

static struct snd_soc_dai_driver dit_stub_dai = {
	.name		= "dog",
	.playback 	= {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 384,
		.rates		= STUB_RATES,
		.formats	= STUB_FORMATS,
	},
};

static const struct of_device_id snd_sunxi_codec_ids[] = {
	{ .compatible = "allwinner,sun4i-a10a-codec", .data = (void *)SUN4A},
	{ .compatible = "allwinner,sun4i-a10-codec", .data = (void *)SUN4I},
	{ .compatible = "allwinner,sun5i-a13-codec", .data = (void *)SUN5I},
	{ .compatible = "allwinner,sun7i-a20-codec", .data = (void *)SUN7I},
	{}
};
MODULE_DEVICE_TABLE(of, snd_sunxi_codec_ids);

static int sunxi_codec_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id;
	struct device *dev = &pdev->dev;
	struct card_data *priv;
	struct resource res;
	int ret;

	if (!of_device_is_available(np))
		return -ENODEV;

	of_id = of_match_device(snd_sunxi_codec_ids, &pdev->dev);
	if (!of_id)
		return -EINVAL;
#ifdef JDS
	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "could not allocate DAI object\n");
		return -ENOMEM;
	}
	
	priv->id = (int)of_id->data;

	/* Get the addresses and IRQ */
	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		dev_err(&pdev->dev, "could not determine device resources\n");
		return ret;
	}
	priv->baseaddr = of_iomap(np, 0);
	if (!priv->baseaddr) {
		dev_err(&pdev->dev, "could not map device resources\n");
		return -ENOMEM;
	}
	priv->codec_phys = res.start;

	priv->irq = irq_of_parse_and_map(np, 0);
	if (!priv->irq) {
		dev_err(&pdev->dev, "no irq for node %s\n", np->full_name);
		return -ENXIO;
	}

	/* Clock */
	priv->codec_apbclk = devm_clk_get(dev, "apb");
	if (IS_ERR(priv->codec_apbclk)) {
		dev_err(dev, "failed to get apb clock.\n");
		return PTR_ERR(priv->codec_apbclk);
	}
	priv->codec_pll2clk = devm_clk_get(dev, "pll2");
	if (IS_ERR(priv->codec_pll2clk)) {
		dev_err(dev, "failed to get pll2 clock.\n");
		return PTR_ERR(priv->codec_pll2clk);
	}
	priv->codec_moduleclk = devm_clk_get(dev, "codec");
	if (IS_ERR(priv->codec_moduleclk)) {
		dev_err(dev, "failed to get codec clock.\n");
		return PTR_ERR(priv->codec_moduleclk);
	}
	ret = clk_set_rate(priv->codec_pll2clk, 24576000);
	if (ret) {
		dev_err(dev, "set codec base clock failed!\n");
		return ret;
	}
	if (clk_prepare_enable(priv->codec_pll2clk)) {
		dev_err(dev, "try to enable codec_pll2clk failed\n");
		return -EINVAL;
	}
	if (clk_prepare_enable(priv->codec_apbclk)) {
		dev_err(dev, "try to enable apb_codec_clk failed\n");
		return -EINVAL;
	}
	if (clk_prepare_enable(priv->codec_moduleclk)) {
		dev_err(dev, "try to enable codec failed\n");
		ret = -EINVAL;
		goto exit_clkdisable_apb_clk;
	}
	dev_set_drvdata(&pdev->dev, priv);

	ret = devm_snd_soc_register_component(&pdev->dev, &sunxi_component, &sunxi_dai, 1);
	if (ret)
		return ret;

	ret = snd_soc_register_codec(&pdev->dev, &soc_codec_sunxi_codec, &dit_stub_dai, 1);
	if (ret)
		return ret;
#endif
//JDS	ret =  devm_sunxi_pcm_platform_register(pdev);
	printk("JDS - codec driver success registered\n");
	return ret;

exit_clkdisable_apb_clk:
	return ret;
}

#ifdef CONFIG_PM
static int snd_sunxi_codec_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sunxi_dev *host = dev_get_drvdata(&pdev->dev);

	if (host->running)
		clk_disable(host->clk);

	return 0;
}

static int snd_sunxi_codec_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sunxi_dev *host = dev_get_drvdata(&pdev->dev);

	if (host->running) {
		clk_enable(host->clk);
		sunxi_configure(host);
		sunxi_clock(host, host->saved_params.core_freq,
				host->saved_params.rate);
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(sunxi_dev_pm_ops, snd_sunxi_codec_suspend, \
		snd_sunxi_codec_resume);

#define sunxi_DEV_PM_OPS (&sunxi_dev_pm_ops)

#else
#define sunxi_DEV_PM_OPS NULL

#endif

static int sunxi_codec_remove(struct platform_device *devptr)
{
#ifdef JDS
	clk_disable(codec_moduleclk);
	//释放codec_pll2clk时钟句柄
	clk_put(codec_pll2clk);
	//释放codec_apbclk时钟句柄
	clk_put(codec_apbclk);

	snd_card_free(platform_get_drvdata(devptr));
	platform_set_drvdata(devptr, NULL);
#endif
	return 0;
}

static void sunxi_codec_shutdown(struct platform_device *devptr)
{
#ifdef JDS
	if (gpio_pa_shutdown) {
//JDS		gpio_write_one_pin_value(gpio_pa_shutdown, 0, "audio_pa_ctrl");
		mdelay(50);
	}
	codec_wr_control(SUNXI_ADC_ACTL, 0x1, PA_ENABLE, 0x0);
	mdelay(100);
	//pa mute
	codec_wr_control(SUNXI_DAC_ACTL, 0x1, PA_MUTE, 0x0);
	mdelay(500);
	//disable dac analog
	codec_wr_control(SUNXI_DAC_ACTL, 0x1, DACAEN_L, 0x0);
	codec_wr_control(SUNXI_DAC_ACTL, 0x1, DACAEN_R, 0x0);

	//disable dac to pa
	codec_wr_control(SUNXI_DAC_ACTL, 0x1, DACPAS, 0x0);
	codec_wr_control(SUNXI_DAC_DPC, 0x1, DAC_EN, 0x0);

	clk_disable(codec_moduleclk);
#endif
}

/*method relating*/
static struct platform_driver sunxi_codec_driver =
{
	.probe = sunxi_codec_probe,
	.remove = sunxi_codec_remove,
	.shutdown = sunxi_codec_shutdown,
#ifdef CONFIG_PM
//JDS	.suspend = snd_sunxi_codec_suspend,
//JDS	.resume = snd_sunxi_codec_resume,
#endif
	.driver = {
		.name = "sunxi-codec",
		.of_match_table = snd_sunxi_codec_ids,
	},
};

module_platform_driver(sunxi_codec_driver);

MODULE_ALIAS("platform:sunxi-codec-dai");
MODULE_DESCRIPTION("sunxi CODEC ALSA codec driver");
MODULE_AUTHOR("software");
MODULE_LICENSE("GPL v2");

