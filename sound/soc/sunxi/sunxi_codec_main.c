/*
 *
 * Licensed under the GPL-2.
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

//Codec Register
#define SUNXI_DAC_DPC                (0x00)
#define SUNXI_DAC_FIFOC              (0x04)
#define SUNXI_DAC_FIFOS              (0x08)
#define SUNXI_DAC_TXDATA             (0x0c)
#define SUNXI_DAC_ACTL               (0x10)
#define SUNXI_DAC_TUNE               (0x14)
#define SUNXI_DAC_DEBUG              (0x18)
#define SUNXI_ADC_FIFOC              (0x1c)
#define SUNXI_ADC_FIFOS              (0x20)
#define SUNXI_ADC_RXDATA             (0x24)
#define SUNXI_ADC_ACTL               (0x28)
#define SUNXI_ADC_DEBUG              (0x2c)
#define SUNXI_DAC_TXCNT              (0x30)
#define SUNXI_ADC_RXCNT              (0x34)
#define SUNXI_BIAS_CRT               (0x38)
#define SUNXI_MIC_CRT                (0x3c)
#define SUNXI_CODEC_REGS_NUM         (13)

#define DAIFMT_16BITS             (16)
#define DAIFMT_20BITS             (20)

#define DAIFMT_BS_MASK            (~(1<<16))  	//FIFO big small mode mask
#define DAIFMT_BITS_MASK          (~(1<<5))		//FIFO Bits select mask,not used yet.
#define SAMPLE_RATE_MASK          (~(7<<29))  	//Sample Rate slect mask

#define DAC_EN                    (31)
#define DIGITAL_VOL               (12)
//For CODEC OLD VERSION
#define DAC_VERSION               (23)

#define DAC_CHANNEL		  (6)
#define LAST_SE                   (26)
#define TX_FIFO_MODE              (24)
#define DRA_LEVEL                 (21)
#define TX_TRI_LEVEL              (8)
#define DAC_MODE                  (6)			//not used yet
#define TASR                      (5)			//not used yet
#define DAC_DRQ                   (4)
#define DAC_FIFO_FLUSH            (0)

#define VOLUME                    (0)
#define PA_MUTE                   (6)
#define MIXPAS                    (7)
#define DACPAS                    (8)
#define MIXEN                     (29)
#define DACAEN_L                  (30)
#define DACAEN_R                  (31)

#define ADC_DIG_EN                (28)
#define RX_FIFO_MODE              (24)
#define RX_TRI_LEVEL              (8)
#define ADC_MODE                  (7)
#define RASR                      (6)
#define ADC_DRQ                   (4)
#define ADC_FIFO_FLUSH            (0)

#define  ADC_LF_EN                (31)
#define  ADC_RI_EN                (30)
#define  ADC_EN                   (30)
#define  MIC1_EN                  (29)
#define  MIC2_EN                  (28)
#define  VMIC_EN                  (27)
#define  MIC_GAIN                 (25)
#define  ADC_SELECT               (17)
#define  PA_ENABLE                (4)
#define  HP_DIRECT                (3)


enum sunxi_device_id {SUN4A, SUN4I, SUN5I, SUN7I}; 

struct sunxi_codec {
	struct regmap *regmap;
	int irq;
	struct clk *clk_apb, *clk_pll2, *clk_module;

	enum sunxi_device_id id;

	struct snd_dmaengine_dai_dma_data playback_dma_data;
	struct snd_dmaengine_dai_dma_data capture_dma_data;

	struct snd_ratnum ratnum;
	struct snd_pcm_hw_constraint_ratnums rate_constraints;
};

static int codec_play_start(struct sunxi_codec *codec)
{
#ifdef JDS
	if (gpio_pa_shutdown)
		gpio_write_one_pin_value(gpio_pa_shutdown, 1, "audio_pa_ctrl");
#endif
	//flush TX FIFO
	regmap_update_bits(codec->regmap, SUNXI_DAC_FIFOC, 0x1 << DAC_FIFO_FLUSH, 0x1 << DAC_FIFO_FLUSH);
	//enable dac drq
	regmap_update_bits(codec->regmap, SUNXI_DAC_FIFOC, 0x1 << DAC_DRQ, 0x1 << DAC_DRQ);
	return 0;
}

static int codec_play_stop(struct sunxi_codec *codec)
{
	//pa mute
#ifdef JDS
	if (gpio_pa_shutdown)
		gpio_write_one_pin_value(gpio_pa_shutdown, 0, "audio_pa_ctrl");
#endif
	regmap_update_bits(codec->regmap, SUNXI_DAC_ACTL, 0x1 << PA_MUTE, 0x0 << PA_MUTE);
	mdelay(5);
	//disable dac drq
	regmap_update_bits(codec->regmap, SUNXI_DAC_FIFOC, 0x1 << DAC_DRQ, 0x0 << DAC_DRQ);
	//pa mute
	regmap_update_bits(codec->regmap, SUNXI_DAC_ACTL, 0x1 << PA_MUTE, 0x0 << PA_MUTE);
	regmap_update_bits(codec->regmap, SUNXI_DAC_ACTL, 0x1 << DACAEN_L, 0x0 << DACAEN_L);
	regmap_update_bits(codec->regmap, SUNXI_DAC_ACTL, 0x1 << DACAEN_R, 0x0 << DACAEN_R);
	return 0;
}

static int codec_capture_start(struct sunxi_codec *codec)
{
	//enable adc drq
#ifdef JDS
	if (gpio_pa_shutdown)
		gpio_write_one_pin_value(gpio_pa_shutdown, 1, "audio_pa_ctrl");
#endif
	regmap_update_bits(codec->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_DRQ, 0x1 << ADC_DRQ);
	return 0;
}

static int codec_capture_stop(struct sunxi_codec *codec)
{
	//disable adc drq
	regmap_update_bits(codec->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_DRQ, 0x0 << ADC_DRQ);
	//enable mic1 pa
	regmap_update_bits(codec->regmap, SUNXI_ADC_ACTL, 0x1 << MIC1_EN, 0x0 << MIC1_EN);
	//enable VMIC
	regmap_update_bits(codec->regmap, SUNXI_ADC_ACTL, 0x1 << VMIC_EN, 0x0 << VMIC_EN);
	if (codec->id == SUN7I) {
		regmap_update_bits(codec->regmap, SUNXI_DAC_TUNE, 0x3 << 8, 0x0 << 8);
	}
	//enable adc digital
	regmap_update_bits(codec->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_DIG_EN, 0x0 << ADC_DIG_EN);
	//set RX FIFO mode
	regmap_update_bits(codec->regmap, SUNXI_ADC_FIFOC, 0x1 << RX_FIFO_MODE, 0x0 << RX_FIFO_MODE);
	//flush RX FIFO
	regmap_update_bits(codec->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_FIFO_FLUSH, 0x0 << ADC_FIFO_FLUSH);
	//enable adc1 analog
	regmap_update_bits(codec->regmap, SUNXI_ADC_ACTL, 0x3 << ADC_EN, 0x0 << ADC_EN);
	return 0;
}

static int sunxi_codec_trigger(struct snd_pcm_substream *substream, int cmd,
	struct snd_soc_dai *dai)
{
	struct sunxi_codec *codec = snd_soc_dai_get_drvdata(dai);

	printk("JDS - sunxi_codec_trigger cmd %d\n", cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			codec_capture_start(codec);
		else
			codec_play_start(codec);
		break;		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			codec_capture_stop(codec);
		else
			codec_play_stop(codec);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sunxi_codec_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct sunxi_codec *codec = snd_soc_dai_get_drvdata(dai);

	printk("JDS - sunxi_codec_prepare\n");
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
		regmap_update_bits(codec->regmap, SUNXI_DAC_DPC, 0x1 << DAC_EN, 0x1 << DAC_EN);
		regmap_update_bits(codec->regmap, SUNXI_DAC_FIFOC, 0x1 << DAC_FIFO_FLUSH, 0x1 << DAC_FIFO_FLUSH);
		//set TX FIFO send drq level
		regmap_update_bits(codec->regmap, SUNXI_DAC_FIFOC, 0xf << TX_TRI_LEVEL, 0xf << TX_TRI_LEVEL);
		if (substream->runtime->rate > 32000) {
			regmap_update_bits(codec->regmap, SUNXI_DAC_FIFOC, 0x1 << 28, 0x0 << 28);
		} else {
			regmap_update_bits(codec->regmap, SUNXI_DAC_FIFOC, 0x1 << 28, 0x1 << 28);
		}
		//set TX FIFO MODE
		regmap_update_bits(codec->regmap, SUNXI_DAC_FIFOC, 0x1 << TX_FIFO_MODE, 0x1 << TX_FIFO_MODE);
		//send last sample when dac fifo under run
		regmap_update_bits(codec->regmap, SUNXI_DAC_FIFOC, 0x1 << LAST_SE, 0x0 << LAST_SE);
		//enable dac analog
		regmap_update_bits(codec->regmap, SUNXI_DAC_ACTL, 0x1 << DACAEN_L, 0x1 << DACAEN_L);
		regmap_update_bits(codec->regmap, SUNXI_DAC_ACTL, 0x1 << DACAEN_R, 0x1 << DACAEN_R);
		//enable dac to pa
		regmap_update_bits(codec->regmap, SUNXI_DAC_ACTL, 0x1 << DACPAS, 0x1 << DACPAS);
	} else {
		//enable mic1 pa
		regmap_update_bits(codec->regmap, SUNXI_ADC_ACTL, 0x1 << MIC1_EN, 0x1 << MIC1_EN);
		//mic1 gain 32dB
		regmap_update_bits(codec->regmap, SUNXI_ADC_ACTL, 0x3 << 25, 0x1 << 25);
		//enable VMIC
		regmap_update_bits(codec->regmap, SUNXI_ADC_ACTL, 0x1 << VMIC_EN, 0x1 << VMIC_EN);

		if (codec->id == SUN7I) {
			/* boost up record effect */
			regmap_update_bits(codec->regmap, SUNXI_DAC_TUNE, 0x3 << 8, 0x1 << 8);
		}

		//enable adc digital
		regmap_update_bits(codec->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_DIG_EN, 0x1 << ADC_DIG_EN);
		//set RX FIFO mode
		regmap_update_bits(codec->regmap, SUNXI_ADC_FIFOC, 0x1 << RX_FIFO_MODE, 0x1 << RX_FIFO_MODE);
		//flush RX FIFO
		regmap_update_bits(codec->regmap, SUNXI_ADC_FIFOC, 0x1 << ADC_FIFO_FLUSH, 0x1 << ADC_FIFO_FLUSH);
		//set RX FIFO rec drq level
		regmap_update_bits(codec->regmap, SUNXI_ADC_FIFOC, 0xf << RX_TRI_LEVEL, 0x7 << RX_TRI_LEVEL);
		//enable adc1 analog
		regmap_update_bits(codec->regmap, SUNXI_ADC_ACTL, 0x3 << ADC_EN, 0x3 << ADC_EN);
	}
	return 0;
}

static int sunxi_codec_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct sunxi_codec *codec = snd_soc_dai_get_drvdata(dai);
	unsigned int rate = params_rate(params);

	printk("JDS - sunxi_codec_hw_params\n");
	switch (params_rate(params)) {
	case 44100:
	case 22050:
	case 11025:
	default:
		clk_set_rate(codec->clk_pll2, 22579200);
		clk_set_rate(codec->clk_module, 22579200);
		break;
	case 192000:
	case 96000:
	case 48000:
	case 32000:
	case 24000:
	case 16000:
	case 12000:
	case 8000:
		clk_set_rate(codec->clk_pll2, 24576000);
		clk_set_rate(codec->clk_module, 24576000);
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
		regmap_update_bits(codec->regmap, SUNXI_DAC_FIFOC, 7 << 29, rate << 29);
		if (substream->runtime->channels == 1)
			regmap_update_bits(codec->regmap, SUNXI_DAC_FIFOC, 1 << 6, 1 << 6);
		else
			regmap_update_bits(codec->regmap, SUNXI_DAC_FIFOC, 1 << 6, 0 << 6);
	} else  {
		regmap_update_bits(codec->regmap, SUNXI_ADC_FIFOC, 7 << 29, rate << 29);
		if (substream->runtime->channels == 1)
			regmap_update_bits(codec->regmap, SUNXI_ADC_FIFOC, 1 << 7, 1 << 7);
		else
			regmap_update_bits(codec->regmap, SUNXI_ADC_FIFOC, 1 << 7, 0 << 7);
	}
	return 0;
}

static int sunxi_codec_dai_probe(struct snd_soc_dai *dai)
{
	struct sunxi_codec *codec = snd_soc_dai_get_drvdata(dai);

	printk("JDS - sunxi_codec_dai_probe\n");
	snd_soc_dai_init_dma_data(dai, &codec->playback_dma_data, &codec->capture_dma_data);

	return 0;
}

static int sunxi_codec_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct sunxi_codec *codec = snd_soc_dai_get_drvdata(dai);
	int ret;
	printk("JDS - sunxi_codec_startup %p %p\n", codec, codec->clk_module);
#ifdef JDS

	ret = snd_pcm_hw_constraint_ratnums(substream->runtime, 0,
			   SNDRV_PCM_HW_PARAM_RATE,
			   &codec->rate_constraints);
	if (ret)
		return ret;

	regmap_update_bits(codec->regmap, SUNXI_SPDIF_REG_CTRL, SUNXI_SPDIF_CTRL_TXEN, SUNXI_SPDIF_CTRL_TXEN);
#endif
	ret = clk_prepare_enable(codec->clk_module);
	if (ret)
		return ret;

	printk("JDS - sunxi_codec_startup - ok\n");
	return 0;
}

static void sunxi_codec_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct sunxi_codec *codec = snd_soc_dai_get_drvdata(dai);

//JDS	regmap_update_bits(codec->regmap, SUNXI_SPDIF_REG_CTRL, SUNXI_SPDIF_CTRL_TXEN, 0);

	printk("JDS - sunxi_codec_shutdown\n");
	clk_disable_unprepare(codec->clk_module);
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
	const struct of_device_id *of_id;
	struct device *dev = &pdev->dev;
	struct sunxi_codec *codec;
	struct resource *res;
	void __iomem *base;
	int ret;

	if (!of_device_is_available(np))
		return -ENODEV;

	of_id = of_match_device(sunxi_codec_of_match, dev);
	if (!of_id)
		return -EINVAL;

	codec = devm_kzalloc(&pdev->dev, sizeof(*codec), GFP_KERNEL);
	if (!codec)
		return -ENOMEM;

	platform_set_drvdata(pdev, codec);

	codec->id = (int)of_id->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	codec->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					    &sunxi_codec_regmap_config);
	if (IS_ERR(codec->regmap))
		return PTR_ERR(codec->regmap);

	codec->irq = irq_of_parse_and_map(np, 0);
	if (!codec->irq) {
		dev_err(dev, "no irq for node %s\n", np->full_name);
		return -ENXIO;
	}

	/* Clock */
	codec->clk_apb = devm_clk_get(dev, "apb");
	if (IS_ERR(codec->clk_apb)) {
		dev_err(dev, "failed to get apb clock.\n");
		return PTR_ERR(codec->clk_apb);
	}
	codec->clk_pll2 = devm_clk_get(dev, "pll2");
	if (IS_ERR(codec->clk_pll2)) {
		dev_err(dev, "failed to get pll2 clock.\n");
		return PTR_ERR(codec->clk_pll2);
	}
	codec->clk_module = devm_clk_get(dev, "codec");
	if (IS_ERR(codec->clk_module)) {
		dev_err(dev, "failed to get codec clock.\n");
		return PTR_ERR(codec->clk_module);
	}

	ret = clk_set_rate(codec->clk_pll2, 24576000);
	if (ret) {
		dev_err(dev, "set codec base clock rate failed!\n");
		return ret;
	}
	if (clk_prepare_enable(codec->clk_pll2)) {
		dev_err(dev, "try to enable clk_pll2 failed\n");
		return -EINVAL;
	}
	if (clk_prepare_enable(codec->clk_apb)) {
		dev_err(dev, "try to enable clk_apb failed\n");
		return -EINVAL;
	}

	codec->playback_dma_data.addr = res->start + SUNXI_DAC_TXDATA;
	codec->playback_dma_data.maxburst = 4;
	codec->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;

	codec->capture_dma_data.addr = res->start + SUNXI_ADC_RXDATA;
	codec->capture_dma_data.maxburst = 4;
	codec->capture_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;

//	printk("JDS - clk_get_rate(codec->clk_module) %lu\n", clk_get_rate(codec->clk_module));
//	codec->ratnum.num = clk_get_rate(codec->clk_module) / 128;
	codec->ratnum.den_step = 1;
	codec->ratnum.den_min = 1;
	codec->ratnum.den_max = 64;

	codec->rate_constraints.rats = &codec->ratnum;
	codec->rate_constraints.nrats = 1;

	ret = devm_snd_soc_register_component(&pdev->dev, &sunxi_codec_component,
					 &sunxi_codec_dai, 1);
	if (ret)
		goto err_clk_disable;

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
	if (ret)
		goto err_clk_disable;

	return 0;

err_clk_disable:
	clk_disable_unprepare(codec->clk_module);
	clk_disable_unprepare(codec->clk_apb);
	clk_disable_unprepare(codec->clk_pll2);
	return ret;
}

static int sunxi_codec_dev_remove(struct platform_device *pdev)
{
	struct sunxi_codec *codec = platform_get_drvdata(pdev);

	clk_disable_unprepare(codec->clk_module);
	clk_disable_unprepare(codec->clk_apb);
	clk_disable_unprepare(codec->clk_pll2);

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

MODULE_ALIAS("platform:sunxi-codec-dai");
MODULE_DESCRIPTION("sunxi CODEC ALSA codec driver");
MODULE_AUTHOR("software");
MODULE_LICENSE("GPL v2");

