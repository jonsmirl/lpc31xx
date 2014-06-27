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
#include <linux/regmap.h>
#include <sound/dmaengine_pcm.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/initval.h>
#include "sunxi-codec.h"

static void codec_resume_events(struct work_struct *work);
struct workqueue_struct *resume_work_queue;
static DECLARE_WORK(codec_resume_work, codec_resume_events);

static void sunxi_configure(struct card_data *priv)
{
#ifdef JDS
	writel(sunxi_RESET, priv->io_base + sunxi_SOFT_RST);
	mdelay(1);
	writel(readl(priv->io_base + sunxi_SOFT_RST) & ~sunxi_RESET,
			priv->io_base + sunxi_SOFT_RST);

	writel(sunxi_FDMA_TRIG_16 | sunxi_MEMFMT_16_16 |
			sunxi_VALID_HW | sunxi_USER_HW |
			sunxi_CHNLSTA_HW | sunxi_PARITY_HW,
			priv->io_base + sunxi_CFG);

	writel(0x7F, priv->io_base + sunxi_INT_STA_CLR);
	writel(0x7F, priv->io_base + sunxi_INT_EN_CLR);
#endif 
}

static int sunxi_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *cpu_dai)
{
	struct card_data *priv = snd_soc_dai_get_drvdata(cpu_dai);
	int ret;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	ret = clk_enable(priv->codec_moduleclk);
	if (ret)
		return ret;

	priv->running = true;
	sunxi_configure(priv);

	return 0;
}

static void sunxi_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct card_data *priv = snd_soc_dai_get_drvdata(dai);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return;

	clk_disable(priv->codec_moduleclk);
	priv->running = false;
}

static void sunxi_clock(struct card_data *priv, u32 core_freq,
		u32 rate)
{
#ifdef JDS
	u32 divider, ctrl;
	clk_set_rate(priv->clk, core_freq);
	divider = DIV_ROUND_CLOSEST(clk_get_rate(priv->clk), (rate * 128));

	ctrl = readl(priv->io_base + sunxi_CTRL);
	ctrl &= ~codec_DIVIDER_MASK;
	ctrl |= (divider << codec_DIVIDER_SHIFT) & codec_DIVIDER_MASK;
	writel(ctrl, priv->io_base + sunxi_CTRL);
#endif
}

static int sunxi_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct card_data *priv = snd_soc_dai_get_drvdata(dai);
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

	sunxi_clock(priv, core_freq, rate);
	priv->saved_params.core_freq = core_freq;
	priv->saved_params.rate = rate;

	return 0;
}

static int sunxi_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
#ifdef JDS
	struct card_data *priv = snd_soc_dai_get_drvdata(dai);
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
			ctrl = readl(priv->io_base + sunxi_CTRL);
			ctrl &= ~codec_OPMODE_MASK;
			if (!priv->saved_params.mute)
				ctrl |= codec_OPMODE_AUD_DATA |
					codec_STATE_NORMAL;
			else
				ctrl |= codec_OPMODE_MUTE_PCM;
			writel(ctrl, priv->io_base + sunxi_CTRL);
#endif
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
#ifdef JDS		
		ctrl = readl(priv->io_base + sunxi_CTRL);
		ctrl &= ~codec_OPMODE_MASK;
		ctrl |= codec_OPMODE_OFF;
		writel(ctrl, priv->io_base + sunxi_CTRL);
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
	struct card_data *priv = snd_soc_dai_get_drvdata(dai);
	u32 val;

	priv->saved_params.mute = mute;
	val = readl(priv->io_base + sunxi_CTRL);
	val &= ~codec_OPMODE_MASK;

	if (mute)
		val |= codec_OPMODE_MUTE_PCM;
	else {
		if (priv->running)
			val |= codec_OPMODE_AUD_DATA | codec_STATE_NORMAL;
		else
			val |= codec_OPMODE_OFF;
	}

	writel(val, priv->io_base + sunxi_CTRL);
#endif
	return 0;
}

static int codec_mute_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct card_data *priv = snd_soc_dai_get_drvdata(cpu_dai);

	ucontrol->value.integer.value[0] = priv->saved_params.mute;
	return 0;
}

static int codec_mute_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct card_data *priv = snd_soc_dai_get_drvdata(cpu_dai);

	if (priv->saved_params.mute == ucontrol->value.integer.value[0])
		return 0;

	sunxi_digital_mute(cpu_dai, ucontrol->value.integer.value[0]);

	return 1;
}

#ifdef JDS
static void codec_resume_events(struct work_struct *work)
{
	printk("%s,%d\n", __func__, __LINE__);

	if (priv->id == SUN7I) {
		regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1, PA_MUTE, 0x0);
	else
		regmap_update_bits(priv->regmap, SUNXI_DAC_DPC, 0x1, DAC_EN, 0x1);

	msleep(20);
	//enable PA
	regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x1, PA_ENABLE, 0x1);
	msleep(550);
	//enable dac analog

	if (priv->id == SUN7I) {
		regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1, PA_MUTE, 0x1);
		regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x1, 8, 0x0);
	} else {
		regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1, DACAEN_L, 0x1);
		regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1, DACAEN_R, 0x1);

		regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1, DACPAS, 0x1);
	}
	if (gpio_pa_shutdown) {
		msleep(50);
		gpio_write_one_pin_value(gpio_pa_shutdown, 1, "audio_pa_ctrl");
	}
}
#endif



/**
 *	codec_reset - reset the codec
 * @codec	SoC Audio Codec
 * Reset the codec, set the register of codec default value
 * Return 0 for success
 */
static int codec_init(struct card_data *priv)
{
	//enable dac digital
	regmap_update_bits(priv->regmap, SUNXI_DAC_DPC, 0x1 << DAC_EN, 0x1 << DAC_EN);

	regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x1 << 28, 0x1 << 28);
	//set digital volume to maximum
	if (priv->id == SUN4A)
		regmap_update_bits(priv->regmap, SUNXI_DAC_DPC, 0x6 << DIGITAL_VOL, 0x0 << DIGITAL_VOL);

	//pa mute
	regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x1 << PA_MUTE, 0x0 << PA_MUTE);
	//enable PA
	regmap_update_bits(priv->regmap, SUNXI_ADC_ACTL, 0x1 << PA_ENABLE, 0x1 << PA_ENABLE);
	regmap_update_bits(priv->regmap, SUNXI_DAC_FIFOC, 0x3 << DRA_LEVEL, 0x3 << DRA_LEVEL);
	//set volume
	if ((priv->id == SUN4A) || (priv->id == SUN4I)) {
		int device_lr_change = 0;
		if (priv->id == SUN4A)
			regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x6 << VOLUME, 0x01 << VOLUME);
		else 
			regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x6 << VOLUME, 0x3b << VOLUME);
#ifdef JDS
		rc = script_parser_fetch("audio_para", "audio_lr_change", &device_lr_change, 1);
		if (rc != SCRIPT_AUDIO_OK) {
			pr_err("No audio_lr_change in fex audio_para\n");
			return -1;
		}
#endif
		if (device_lr_change)
			regmap_update_bits(priv->regmap, SUNXI_DAC_DEBUG, 0x1 << DAC_CHANNEL, 0x1 << DAC_CHANNEL);
	} else {
		regmap_update_bits(priv->regmap, SUNXI_DAC_ACTL, 0x6 << VOLUME, 0x3b << VOLUME);
	}
	return 0;
}


/*	对sunxi-codec.c各寄存器的各种设定，或读取。主要实现函数有三个.
 * 	.info = snd_codec_info_volsw, .get = snd_codec_get_volsw,\.put = snd_codec_put_volsw,
 * It should be noted that the only difference between sunxi and sun5i is the Master Playback Volume
 */
static const struct snd_kcontrol_new sun4i_dac[] = {
	//FOR B C VERSION
	SOC_SINGLE("Master Playback Volume", SUNXI_DAC_ACTL, 0, 0x3f, 0), 
	SOC_SINGLE("Playback Switch", SUNXI_DAC_ACTL, 6, 1, 0), //全局输出开关
	SOC_SINGLE("Fm Volume", SUNXI_DAC_ACTL, 23, 7, 0), //Fm 音量
	SOC_SINGLE("Line Volume", SUNXI_DAC_ACTL, 26, 1, 0), //Line音量
	SOC_SINGLE("FmL Switch", SUNXI_DAC_ACTL, 17, 1, 0), //Fm左开关
	SOC_SINGLE("FmR Switch", SUNXI_DAC_ACTL, 16, 1, 0), //Fm右开关
	SOC_SINGLE("LineL Switch", SUNXI_DAC_ACTL, 19, 1, 0), //Line左开关
	SOC_SINGLE("LineR Switch", SUNXI_DAC_ACTL, 18, 1, 0), //Line右开关
	SOC_SINGLE("Ldac Left Mixer", SUNXI_DAC_ACTL, 15, 1, 0), 
	SOC_SINGLE("Rdac Right Mixer", SUNXI_DAC_ACTL, 14, 1, 0), 
	SOC_SINGLE("Ldac Right Mixer", SUNXI_DAC_ACTL, 13, 1, 0),
	SOC_SINGLE("Mic Input Mux", SUNXI_DAC_ACTL, 9, 15, 0), //from bit 9 to bit 12.Mic（麦克风）输入静音
};

static const struct snd_kcontrol_new sun4a_dac[] = {
	//For A VERSION
	SOC_SINGLE("Master Playback Volume", SUNXI_DAC_DPC, 12, 0x3f, 0), //62 steps, 3e + 1 = 3f 主音量控制
	SOC_SINGLE("Playback Switch", SUNXI_DAC_ACTL, 6, 1, 0), //全局输出开关
	SOC_SINGLE("Fm Volume", SUNXI_DAC_ACTL, 23, 7, 0), //Fm 音量
	SOC_SINGLE("Line Volume", SUNXI_DAC_ACTL, 26, 1, 0), //Line音量
	SOC_SINGLE("FmL Switch", SUNXI_DAC_ACTL, 17, 1, 0), //Fm左开关
	SOC_SINGLE("FmR Switch", SUNXI_DAC_ACTL, 16, 1, 0), //Fm右开关
	SOC_SINGLE("LineL Switch", SUNXI_DAC_ACTL, 19, 1, 0), //Line左开关
	SOC_SINGLE("LineR Switch", SUNXI_DAC_ACTL, 18, 1, 0), //Line右开关
	SOC_SINGLE("Ldac Left Mixer", SUNXI_DAC_ACTL, 15, 1, 0), 
	SOC_SINGLE("Rdac Right Mixer", SUNXI_DAC_ACTL, 14, 1, 0), 
	SOC_SINGLE("Ldac Right Mixer", SUNXI_DAC_ACTL, 13, 1, 0),
	SOC_SINGLE("Mic Input Mux", SUNXI_DAC_ACTL, 9, 15, 0), //from bit 9 to bit 12.Mic（麦克风）输入静音
};

static const struct snd_kcontrol_new sunxi_adc_controls[] = { 
	SOC_SINGLE("Master Capture Mute", SUNXI_ADC_ACTL, 4, 1, 0), 
	SOC_SINGLE("Right Capture Mute", SUNXI_ADC_ACTL, 31, 1, 0), 
	SOC_SINGLE("Left Capture Mute", SUNXI_ADC_ACTL, 30, 1, 0),
	SOC_SINGLE("Capture Volume", SUNXI_ADC_ACTL, 20, 7, 0), //录音音量
	SOC_SINGLE("Line Capture Volume", SUNXI_ADC_ACTL, 13, 7, 0), 
	SOC_SINGLE("MicL Volume", SUNXI_ADC_ACTL, 25, 3, 0), //mic左音量
	SOC_SINGLE("MicR Volume", SUNXI_ADC_ACTL, 23, 3, 0), //mic右音量
	SOC_SINGLE("Mic2 Boost", SUNXI_ADC_ACTL, 29, 1, 0), 
	SOC_SINGLE("Mic1 Boost", SUNXI_ADC_ACTL, 28, 1, 0), 
	SOC_SINGLE("Mic Power", SUNXI_ADC_ACTL, 27, 1, 0), 
	SOC_SINGLE("ADC Input Mux", SUNXI_ADC_ACTL, 17, 7, 0), //ADC输入静音
};

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
};

static const struct snd_kcontrol_new sun7i_adc_ctls[] = { 
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



static int sunxi_soc_dai_probe(struct snd_soc_dai *dai)
{
	struct card_data *priv = snd_soc_dai_get_drvdata(dai);
	struct snd_dmaengine_dai_dma_data *playback_dma_data;
	struct snd_dmaengine_dai_dma_data *capture_dma_data;

	playback_dma_data = devm_kzalloc(dai->dev,
					 sizeof(*playback_dma_data),
					 GFP_KERNEL);
	if (!playback_dma_data)
		return -ENOMEM;

	capture_dma_data = devm_kzalloc(dai->dev,
					sizeof(*capture_dma_data),
					GFP_KERNEL);
	if (!capture_dma_data)
		return -ENOMEM;

	playback_dma_data->addr = priv->codec_phys + SUNXI_DAC_TXDATA;
	capture_dma_data->addr = priv->codec_phys + SUNXI_ADC_RXDATA;

	playback_dma_data->maxburst = 4;
	capture_dma_data->maxburst = 4;

	playback_dma_data->addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	playback_dma_data->addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;

	snd_soc_dai_init_dma_data(dai, playback_dma_data, capture_dma_data);
	return 0;
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

unsigned int read(struct snd_soc_codec *codec, unsigned int reg);
int write(struct snd_soc_codec *, unsigned int reg, unsigned int);


static struct snd_soc_codec_driver soc_codec_sun4a_codec = {
	.controls = sun4a_dac,
	.num_controls = ARRAY_SIZE(sun4a_dac),
};

static struct snd_soc_codec_driver soc_codec_sun4i_codec = {
	.controls = sun4i_dac,
	.num_controls = ARRAY_SIZE(sun4i_dac),
};

static struct snd_soc_codec_driver soc_codec_sun7i_codec = {
	.controls = sun7i_dac_ctls,
	.num_controls = ARRAY_SIZE(sun7i_dac_ctls),
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

static const struct regmap_config sunxi_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = SUNXI_MIC_CRT,
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

	priv->regmap = devm_regmap_init_mmio(&pdev->dev, priv->baseaddr, &sunxi_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

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

	switch (priv->id) {
	case SUN4A:
		ret = snd_soc_register_codec(&pdev->dev, &soc_codec_sun4a_codec, &dit_stub_dai, 1);
	case SUN4I:
	case SUN5I:
		ret = snd_soc_register_codec(&pdev->dev, &soc_codec_sun4i_codec, &dit_stub_dai, 1);
	case SUN7I:
		ret = snd_soc_register_codec(&pdev->dev, &soc_codec_sun7i_codec, &dit_stub_dai, 1);
	}
	if (ret)
		return ret;

	snd_dmaengine_pcm_register(&pdev->dev, NULL, SND_DMAENGINE_PCM_FLAG_NO_RESIDUE);

	codec_init(priv);
#ifdef JDS
	if (gpio_pa_shutdown)
		gpio_write_one_pin_value(gpio_pa_shutdown, 0, "audio_pa_ctrl");

	resume_work_queue = create_singlethread_workqueue("codec_resume");
	if (resume_work_queue == NULL) {
		printk("[su4i-codec] try to create workqueue for codec failed!\n");
		ret = -ENOMEM;
		goto err_resume_work_queue;
	}
#endif
	printk("JDS - codec driver success registered\n");
	return ret;

err_resume_work_queue:
exit_clkdisable_apb_clk:
	return ret;
}

#ifdef CONFIG_PM
static int snd_sunxi_codec_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct card_data *priv = dev_get_drvdata(&pdev->dev);

	if (priv->running)
		clk_disable(priv->codec_moduleclk);

	return 0;
}

static int snd_sunxi_codec_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct card_data *priv = dev_get_drvdata(&pdev->dev);

	if (priv->running) {
		clk_enable(priv->codec_moduleclk);
		sunxi_configure(priv);
		sunxi_clock(priv, priv->saved_params.core_freq,
				priv->saved_params.rate);
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(sunxi_dev_pm_ops, snd_sunxi_codec_suspend, \
		snd_sunxi_codec_resume);

#define sunxi_DEV_PM_OPS (&sunxi_dev_pm_ops)

#else
#define sunxi_DEV_PM_OPS NULL

#endif

static int sunxi_codec_remove(struct platform_device *pdev)
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
	snd_dmaengine_pcm_unregister(&pdev->dev);
}

static void sunxi_codec_shutdown(struct platform_device *devptr)
{
#ifdef JDS
	if (gpio_pa_shutdown) {
//JDS		gpio_write_one_pin_value(gpio_pa_shutdown, 0, "audio_pa_ctrl");
		mdelay(50);
	}
	regmap_update_bits(SUNXI_ADC_ACTL, 0x1, PA_ENABLE, 0x0);
	mdelay(100);
	//pa mute
	regmap_update_bits(SUNXI_DAC_ACTL, 0x1, PA_MUTE, 0x0);
	mdelay(500);
	//disable dac analog
	regmap_update_bits(SUNXI_DAC_ACTL, 0x1, DACAEN_L, 0x0);
	regmap_update_bits(SUNXI_DAC_ACTL, 0x1, DACAEN_R, 0x0);

	//disable dac to pa
	regmap_update_bits(SUNXI_DAC_ACTL, 0x1, DACPAS, 0x0);
	regmap_update_bits(SUNXI_DAC_DPC, 0x1, DAC_EN, 0x0);

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

