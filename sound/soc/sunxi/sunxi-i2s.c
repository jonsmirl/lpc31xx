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
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/dmaengine_pcm.h>

#include "sunxi-i2s.h"

void sunxi_snd_txctrl_i2s(struct sunxi_priv *priv, struct snd_pcm_substream *substream, int on)
{
	u32 reg_val;

	regmap_update_bits(priv->regmap, SUNXI_I2S_TXCHSEL, SUNXI_I2STXCHSEL_CHNUM_MASK, 
				(substream->runtime->channels - 1) << SUNXI_I2STXCHSEL_CHNUM_SHIFT);

	regmap_raw_read(priv->regmap, SUNXI_I2S_TXCHMAP, &reg_val, sizeof(reg_val));
	reg_val = 0;
	if (priv->revision == SUN4I) {
		if(substream->runtime->channels == 1) {
			reg_val = 0x76543200;
		} else {
			reg_val = 0x76543210;
		}
	} else {
		if(substream->runtime->channels == 1) {
			reg_val = 0x00000000;
		} else {
			reg_val = 0x00000010;
		}
	}
	regmap_write(priv->regmap, SUNXI_I2S_TXCHMAP, reg_val);

	regmap_read(priv->regmap, SUNXI_I2S_CTL, &reg_val);
	if (priv->revision == SUN4I) {
		reg_val &= ~SUNXI_I2SCTL_SDOEN_ALL;
		switch(substream->runtime->channels) {
			case 1:
			case 2:
				reg_val |= SUNXI_I2SCTL_SDO0EN;
				break;
			case 3:
			case 4:
				reg_val |= SUNXI_I2SCTL_SDO0EN;
				reg_val |= SUNXI_I2SCTL_SDO1EN;
				break;
			case 5:
			case 6:
				reg_val |= SUNXI_I2SCTL_SDO0EN;
				reg_val |= SUNXI_I2SCTL_SDO1EN;
				reg_val |= SUNXI_I2SCTL_SDO2EN;
				break;
			case 7:
			case 8:
				reg_val |= SUNXI_I2SCTL_SDO0EN;
				reg_val |= SUNXI_I2SCTL_SDO1EN;
				reg_val |= SUNXI_I2SCTL_SDO2EN;
				reg_val |= SUNXI_I2SCTL_SDO3EN;
				break;
			default:
				reg_val |= SUNXI_I2SCTL_SDO0EN;
		}
	} else {
		reg_val |= SUNXI_I2SCTL_SDO0EN;
	}
	regmap_write(priv->regmap, SUNXI_I2S_CTL, reg_val);

	regmap_read(priv->regmap, SUNXI_I2S_TXCNT, &reg_val);
	printk("JDS - SUNXI_I2S_TXCNT %d\n", reg_val);

	//flush TX FIFO
	regmap_update_bits(priv->regmap, SUNXI_I2S_FCTL, SUNXI_I2SFCTL_FTX_MASK, SUNXI_I2SFCTL_FTX);

	//clear TX counter
	regmap_write(priv->regmap, SUNXI_I2S_TXCNT, 0);

	if (on) {
		/* IIS TX ENABLE */
		regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_TXEN_MASK, SUNXI_I2SCTL_TXEN);

		/* enable DMA DRQ mode for play */
		regmap_update_bits(priv->regmap, SUNXI_I2S_INT, SUNXI_I2SINT_TXDRQEN_MASK, SUNXI_I2SINT_TXDRQEN);

	} else {
		/* IIS TX DISABLE */
		regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_TXEN_MASK, 0);

		/* DISBALE dma DRQ mode */
		regmap_update_bits(priv->regmap, SUNXI_I2S_INT, SUNXI_I2SINT_TXDRQEN_MASK, 0);
	}

	{
	int i;
	for (i = SUNXI_I2S_CTL; i <= SUNXI_I2S_RXCHMAP; i += 4)  {
		regmap_read(priv->regmap, i, &reg_val);
		printk("REG %02x VAL %08x\n", i, reg_val);
	}
	}

}

void sunxi_snd_rxctrl_i2s(struct sunxi_priv *priv, int on)
{
	//flush RX FIFO
	regmap_update_bits(priv->regmap, SUNXI_I2S_FCTL, SUNXI_I2SFCTL_FRX_MASK, SUNXI_I2SFCTL_FRX);

	//clear RX counter
	regmap_write(priv->regmap, SUNXI_I2S_RXCNT, 0);

	if (on) {
		/* IIS RX ENABLE */
		regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_RXEN_MASK, SUNXI_I2SCTL_RXEN);

		/* enable DMA DRQ mode for record */
		regmap_update_bits(priv->regmap, SUNXI_I2S_INT, SUNXI_I2SINT_RXDRQEN_MASK, SUNXI_I2SINT_RXDRQEN);

	} else {
		/* IIS RX DISABLE */
		regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_RXEN_MASK, 0);

		/* DISBALE dma DRQ mode */
		regmap_update_bits(priv->regmap, SUNXI_I2S_INT, SUNXI_I2SINT_RXDRQEN_MASK, 0);
	}
}

static int sunxi_i2s_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct sunxi_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	u32 reg_val;
	int ret;

	printk("JDS - sunxi_i2s_set_fmt %08x\n", fmt);

	//SDO ON
	if (priv->revision == SUN4I) {
		regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_SDOEN_ALL, SUNXI_I2SCTL_SDOEN_ALL);
	} else {
		regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_SDO0EN_MASK, SUNXI_I2SCTL_SDO0EN);
	}


	/* master or slave selection */
	switch(fmt & SND_SOC_DAIFMT_MASTER_MASK){
	case SND_SOC_DAIFMT_CBM_CFM:   /* codec clk & frm master */
		regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_MS_MASK, SUNXI_I2SCTL_SLAVE);
		printk("JDS - sunxi_i2s_set_fmt slave\n");
		priv->master = 0;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:   /* codec clk & frm slave */
		regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_MS_MASK, 0);
		printk("JDS - sunxi_i2s_set_fmt master\n");
		priv->master = 1;
		/* Enable iis on a basic rate */
		ret = clk_set_rate(priv->clk_iis, 24576000);
		if (ret) {
			dev_err(priv->dev, "failed to set i2s base clock rate\n");
			return ret;
		}
		if (clk_prepare_enable(priv->clk_iis)) {
			dev_err(priv->dev, "failed to enable iis clock\n");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	/* pcm or i2s mode selection */
	switch(fmt & SND_SOC_DAIFMT_FORMAT_MASK){
	case SND_SOC_DAIFMT_I2S:        /* I2S mode */
		regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_PCM_MASK, 0);
		regmap_update_bits(priv->regmap, SUNXI_I2S_FAT0, SUNXI_I2SFAT0_FMT_MASK, SUNXI_I2SFAT0_FMT_I2S);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:    /* Right Justified mode */
		regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_PCM_MASK, 0);
		regmap_update_bits(priv->regmap, SUNXI_I2S_FAT0, SUNXI_I2SFAT0_FMT_MASK, SUNXI_I2SFAT0_FMT_RGT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:     /* Left Justified mode */
		regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_PCM_MASK, 0);
		regmap_update_bits(priv->regmap, SUNXI_I2S_FAT0, SUNXI_I2SFAT0_FMT_MASK, SUNXI_I2SFAT0_FMT_LFT);
		break;
	case SND_SOC_DAIFMT_DSP_A:      /* L data msb after FRM LRC */
		regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_PCM_MASK, SUNXI_I2SCTL_PCM);
		regmap_update_bits(priv->regmap, SUNXI_I2S_FAT0, SUNXI_I2SFAT0_LRCP_MASK, 0);
		break;
	case SND_SOC_DAIFMT_DSP_B:      /* L data msb during FRM LRC */
		regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_PCM_MASK, SUNXI_I2SCTL_PCM);
		regmap_update_bits(priv->regmap, SUNXI_I2S_FAT0, SUNXI_I2SFAT0_LRCP_MASK, SUNXI_I2SFAT0_LRCP);
		break;
	default:
		return -EINVAL;
	}

	/* DAI signal inversions */
	switch(fmt & SND_SOC_DAIFMT_INV_MASK){
	case SND_SOC_DAIFMT_NB_NF:     /* normal bit clock + frame */
		regmap_update_bits(priv->regmap, SUNXI_I2S_FAT0, SUNXI_I2SFAT0_LRCP_MASK, 0);
		regmap_update_bits(priv->regmap, SUNXI_I2S_FAT0, SUNXI_I2SFAT0_BCP_MASK, 0);
		break;
	case SND_SOC_DAIFMT_NB_IF:     /* normal bclk + inv frm */
		regmap_update_bits(priv->regmap, SUNXI_I2S_FAT0, SUNXI_I2SFAT0_LRCP_MASK, SUNXI_I2SFAT0_LRCP);
		regmap_update_bits(priv->regmap, SUNXI_I2S_FAT0, SUNXI_I2SFAT0_BCP_MASK, 0);
		break;
	case SND_SOC_DAIFMT_IB_NF:     /* invert bclk + nor frm */
		regmap_update_bits(priv->regmap, SUNXI_I2S_FAT0, SUNXI_I2SFAT0_LRCP_MASK, 0);
		regmap_update_bits(priv->regmap, SUNXI_I2S_FAT0, SUNXI_I2SFAT0_BCP_MASK, SUNXI_I2SFAT0_BCP);
		break;
	case SND_SOC_DAIFMT_IB_IF:     /* invert bclk + frm */
		regmap_update_bits(priv->regmap, SUNXI_I2S_FAT0, SUNXI_I2SFAT0_LRCP_MASK, SUNXI_I2SFAT0_LRCP);
		regmap_update_bits(priv->regmap, SUNXI_I2S_FAT0, SUNXI_I2SFAT0_BCP_MASK, SUNXI_I2SFAT0_BCP);
		break;
	}

	/* PCM REGISTER setup */
	reg_val = priv->pcm_txtype & SUNXI_I2SFAT1_TXPDM_MASK;
	reg_val |= priv->pcm_rxtype << SUNXI_I2SFAT1_RXPDM_SHIFT;

	if(!priv->pcm_sync_type)
		reg_val |= SUNXI_I2SFAT1_SSYNC;				// short sync
	if(priv->pcm_sw == 16)
		reg_val |= SUNXI_I2SFAT1_SW;

	reg_val |= ((priv->pcm_start_slot - 1) << SUNXI_I2SFAT1_SI_SHIFT) & SUNXI_I2SFAT1_SI_MASK; // start slot index

	reg_val |= priv->pcm_lsb_first << SUNXI_I2SFAT1_MLS_SHIFT;	// MSB or LSB first

	if(priv->pcm_sync_period == 256)
		reg_val |= SUNXI_I2SFAT1_SYNCLEN_256BCLK;
	else if (priv->pcm_sync_period == 128)
		reg_val |= SUNXI_I2SFAT1_SYNCLEN_128BCLK;
	else if (priv->pcm_sync_period == 64)
		reg_val |= SUNXI_I2SFAT1_SYNCLEN_64BCLK;
	else if (priv->pcm_sync_period == 32)
		reg_val |= SUNXI_I2SFAT1_SYNCLEN_32BCLK;
	regmap_write(priv->regmap, SUNXI_I2S_FAT1, reg_val);

	/* set FIFO control register */
	reg_val = SUNXI_I2SFCTL_RXOM_MOD0;
	reg_val |= SUNXI_I2SFCTL_TXIM_MOD1;
	reg_val |= SUNXI_I2SFCTL_RXTL(0xf);				//RX FIFO trigger level
	reg_val |= SUNXI_I2SFCTL_TXTL(0x40);				//TX FIFO empty trigger level
	regmap_write(priv->regmap, SUNXI_I2S_FCTL, reg_val);
	return 0;
}


static int sunxi_i2s_hw_params(struct snd_pcm_substream *substream, 
		struct snd_pcm_hw_params *params, struct snd_soc_dai *cpu_dai)
{
	struct sunxi_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned int ret = 0, fmt, div;
	unsigned long rate = params_rate(params);

	/* set i2s data format */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		fmt = SUNXI_I2SFAT0_SR_16BIT | SUNXI_I2SFAT0_WSS_16BCLK;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			priv->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		fmt = SUNXI_I2SFAT0_SR_20BIT | SUNXI_I2SFAT0_WSS_20BCLK;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			priv->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		fmt = SUNXI_I2SFAT0_SR_24BIT | SUNXI_I2SFAT0_WSS_24BCLK;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			priv->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	default:
		return -EINVAL;
	}
	regmap_update_bits(priv->regmap, SUNXI_I2S_FAT0, SUNXI_I2SFAT0_SR_MASK | SUNXI_I2SFAT0_WSS_MASK, fmt);

	div  = SUNXI_I2SCLKD_MCLKDIV_1;
	regmap_update_bits(priv->regmap, SUNXI_I2S_CLKD, SUNXI_I2SCLKD_MCLKDIV_MASK, div);

	printk("JDS - sunxi_i2s_hw_params sysclk %d rate %ld\n", priv->sysclk, rate);

	if (priv->master) {
		/* asssumes 32b i2s format */
		switch (DIV_ROUND_UP(priv->sysclk, rate)) {
		case 256:
			div  = SUNXI_I2SCLKD_BCLKDIV_8;
			break;
		case 384:
			div  = SUNXI_I2SCLKD_BCLKDIV_12;
			break;
		case 512:
			div  = SUNXI_I2SCLKD_BCLKDIV_8;
			break;
		default:
			return -EINVAL;
		}
		regmap_update_bits(priv->regmap, SUNXI_I2S_CLKD, SUNXI_I2SCLKD_BCLKDIV_MASK, div);
	}

	return ret;
}

static int sunxi_i2s_trigger(struct snd_pcm_substream *substream,
                              int cmd, struct snd_soc_dai *cpu_dai)
{
	struct sunxi_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	int ret = 0;

	printk("JDS - sunxi_i2s_trigger %d\n", cmd);
	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
				sunxi_snd_rxctrl_i2s(priv, 1);
			} else {
				sunxi_snd_txctrl_i2s(priv, substream, 1);
			}
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
				sunxi_snd_rxctrl_i2s(priv, 0);
			} else {
				sunxi_snd_txctrl_i2s(priv, substream, 0);
			}
			break;
		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}

static int sunxi_i2s_set_sysclk(struct snd_soc_dai *cpu_dai, int clk_id, unsigned int freq, int dir)
{
	struct sunxi_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);

	if ((freq == 24576000) || (freq == 22579200)) {
		clk_set_rate(priv->clk_iis, freq);
	}
	priv->sysclk = freq;
	return 0;
}

static int sunxi_i2s_dai_probe(struct snd_soc_dai *cpu_dai)
{
	struct sunxi_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);

	snd_soc_dai_init_dma_data(cpu_dai, &priv->playback_dma_data, &priv->capture_dma_data);

	return 0;
}

static int sunxi_i2s_dai_remove(struct snd_soc_dai *cpu_dai)
{
	return 0;
}


static int sunxi_i2s_mclk_prepare(struct clk_hw *hw)
{
	struct sunxi_priv *priv = container_of(hw, struct sunxi_priv, mclk_div.hw);
	int ret;

//	ret = regmap_update_bits(priv->regmap, SUNXI_I2S_CLKD, SUNXI_I2SCLKD_MCLKDIV_MASK, SUNXI_I2SCLKD_MCLKDIV_1);
	ret = regmap_update_bits(priv->regmap, SUNXI_I2S_CLKD, SUNXI_I2SCLKD_MCLKOEN_MASK, SUNXI_I2SCLKD_MCLKOEN);
	printk("JDS - sunxi_i2s_mclk_prepare done\n");
	return 0;
}

static struct clk_ops sunxi_i2s_clk_divider_ops = {
};

static int sunxi_i2s_mclk_init(struct platform_device *pdev, struct sunxi_priv *priv, void __iomem *base)
{
	struct device_node *np = pdev->dev.of_node;
	struct clk_init_data init;
	const char *clk_name = NULL;
	const char *clk_parent = __clk_get_name(priv->clk_iis);
	int ret;
	int flags = 0;

	of_property_read_string(np, "clock-output-names", &clk_name);

	sunxi_i2s_clk_divider_ops = clk_divider_ops;
	sunxi_i2s_clk_divider_ops.prepare = sunxi_i2s_mclk_prepare;

	init.name = clk_name;
	init.ops = &sunxi_i2s_clk_divider_ops;
	init.flags = flags | CLK_IS_BASIC;
	init.parent_names = &clk_parent;
	init.num_parents = 1;

	/* struct clk_divider assignments */
	priv->mclk_div.reg = base + SUNXI_I2S_CLKD;
	priv->mclk_div.shift = SUNXI_I2SCLKD_MCLKDIV_SHIFT;
	priv->mclk_div.width = SUNXI_I2SCLKD_MCLKDIV_WIDTH;
	priv->mclk_div.flags = 0;
	priv->mclk_div.lock = NULL;
	priv->mclk_div.hw.init = &init;
	priv->mclk_div.table = NULL;

	/* register the clock */
	priv->clk_mclk = clk_register(&pdev->dev, &priv->mclk_div.hw);

	if (IS_ERR(priv->clk_mclk)) {
		dev_err(&pdev->dev, "failed to register mclk: %ld\n", PTR_ERR(priv->clk_mclk));
		return PTR_ERR(priv->clk_mclk);
	}

	ret = of_clk_add_provider(np, of_clk_src_simple_get, priv->clk_mclk);
	if (ret)
		return ret;

	printk("JDS - sunxi_i2s_mclk_init %p\n", priv->clk_mclk);

	return 0;
}

static void iisregsave(void)
{
	/*regsave[0] = readl(priv->regs + SUNXI_I2S_CTL);
	regsave[1] = readl(priv->regs + SUNXI_I2S_FAT0);
	regsave[2] = readl(priv->regs + SUNXI_I2S_FAT1);
	regsave[3] = readl(priv->regs + SUNXI_I2S_FCTL) | (0x3<<24);
	regsave[4] = readl(priv->regs + SUNXI_I2S_INT);
	regsave[5] = readl(priv->regs + SUNXI_I2S_CLKD);
	regsave[6] = readl(priv->regs + SUNXI_TXCHSEL);
	regsave[7] = readl(priv->regs + SUNXI_TXCHMAP);*/
}

static void iisregrestore(void)
{
	/*writel(regsave[0], priv->regs + SUNXI_I2S_CTL);
	writel(regsave[1], priv->regs + SUNXI_I2S_FAT0);
	writel(regsave[2], priv->regs + SUNXI_I2S_FAT1);
	writel(regsave[3], priv->regs + SUNXI_I2S_FCTL);
	writel(regsave[4], priv->regs + SUNXI_I2S_INT);
	writel(regsave[5], priv->regs + SUNXI_I2S_CLKD);
	writel(regsave[6], priv->regs + SUNXI_TXCHSEL);
	writel(regsave[7], priv->regs + SUNXI_TXCHMAP);*/
}

static int sunxi_i2s_suspend(struct snd_soc_dai *cpu_dai)
{
	struct sunxi_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);

	printk("[IIS]Entered %s\n", __func__);

	//Global Enable Digital Audio Interface
	regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_GEN_MASK, 0);

	iisregsave();

	//release the module clock
	clk_disable(priv->clk_iis);

	clk_disable(priv->clk_apb);

	//printk("[IIS]PLL2 0x01c20008 = %#x\n", *(volatile int*)0xF1C20008);
	printk("[IIS]SPECIAL CLK 0x01c20068 = %#x, line= %d\n", *(volatile int*)0xF1C20068, __LINE__);
	printk("[IIS]SPECIAL CLK 0x01c200B8 = %#x, line = %d\n", *(volatile int*)0xF1C200B8, __LINE__);

	return 0;
}
static int sunxi_i2s_resume(struct snd_soc_dai *cpu_dai)
{
	struct sunxi_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);

	printk("[IIS]Entered %s\n", __func__);

	//release the module clock
	clk_enable(priv->clk_apb);

	//release the module clock
	clk_enable(priv->clk_iis);

	iisregrestore();

	//Global Enable Digital Audio Interface
	regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_GEN_MASK, SUNXI_I2SCTL_GEN);

	//printk("[IIS]PLL2 0x01c20008 = %#x\n", *(volatile int*)0xF1C20008);
	printk("[IIS]SPECIAL CLK 0x01c20068 = %#x, line= %d\n", *(volatile int*)0xF1C20068, __LINE__);
	printk("[IIS]SPECIAL CLK 0x01c200B8 = %#x, line = %d\n", *(volatile int*)0xF1C200B8, __LINE__);

	return 0;
}

#define SUNXI_I2S_RATES (SNDRV_PCM_RATE_8000_192000 | SNDRV_PCM_RATE_KNOT)
static struct snd_soc_dai_ops sunxi_i2s_dai_ops = {
	.trigger 	= sunxi_i2s_trigger,
	.hw_params 	= sunxi_i2s_hw_params,
	.set_fmt 	= sunxi_i2s_set_fmt,
	.set_sysclk	= sunxi_i2s_set_sysclk,
};

static struct snd_soc_dai_driver sunxi_i2s_dai = {
	.probe 		= sunxi_i2s_dai_probe,
	.suspend 	= sunxi_i2s_suspend,
	.resume 	= sunxi_i2s_resume,
	.remove 	= sunxi_i2s_dai_remove,
	.playback 	= {
		.channels_min = 1,
		.channels_max = 8,
		.rates = SUNXI_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture 	= {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SUNXI_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.symmetric_rates = 1,
	.ops 		= &sunxi_i2s_dai_ops,
};

static const struct snd_soc_component_driver sunxi_i2s_component = {
	.name	= "sunxi-i2s",
};

static const struct regmap_range sunxi_i2s_volatile_regs_range[] = {
/*	regmap_reg_range(SUNXI_I2S_TXFIFO, SUNXI_I2S_FCTL),
	regmap_reg_range(SUNXI_I2S_ISTA, SUNXI_I2S_ISTA),
	regmap_reg_range(SUNXI_I2S_TXCNT, SUNXI_I2S_RXCNT), */
	regmap_reg_range(SUNXI_I2S_CTL, SUNXI_I2S_RXCHMAP),
};

static const struct regmap_access_table sunxi_i2s_volatile_regs = {
	.yes_ranges	= sunxi_i2s_volatile_regs_range,
	.n_yes_ranges	= ARRAY_SIZE(sunxi_i2s_volatile_regs_range),
};

static const struct regmap_config sunxi_i2s_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register 	= SUNXI_I2S_RXCHMAP,
	.volatile_table	= &sunxi_i2s_volatile_regs,
	.fast_io	= true,
};

static const struct of_device_id sunxi_i2s_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-iis", .data = (void *)SUN4I},
	{ .compatible = "allwinner,sun5i-a13-iis", .data = (void *)SUN5I},
	{ .compatible = "allwinner,sun7i-a20-iis", .data = (void *)SUN7I},
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

	printk("JDS: sunxi_i2s_probe pdev %p\n", pdev);
	of_id = of_match_device(sunxi_i2s_of_match, dev);
	if (!of_id)
		return -EINVAL;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->revision = (enum sunxi_soc_family)of_id->data;
	priv->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->regmap = devm_regmap_init_mmio(dev, base,
					     &sunxi_i2s_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	/* Get the clocks from the DT */
	priv->clk_apb = devm_clk_get(dev, "apb");
	if (IS_ERR(priv->clk_apb)) {
		dev_err(dev, "failed to get apb clock\n");
		return PTR_ERR(priv->clk_apb);
	}
	priv->clk_iis = devm_clk_get(dev, "iis");
	if (IS_ERR(priv->clk_iis)) {
		dev_err(dev, "failed to get iis clock\n");
		return PTR_ERR(priv->clk_iis);
	}
#ifdef JDS
	/* Enable iis on a basic rate */
	ret = clk_set_rate(priv->clk_iis, 24576000);
	if (ret) {
		dev_err(dev, "failed to set i2s base clock rate\n");
		return ret;
	}
	if (clk_prepare_enable(priv->clk_iis)) {
		dev_err(dev, "failed to enable iis clock\n");
		return -EINVAL;
	}
#endif
	/* Enable the bus clock */
	if (clk_prepare_enable(priv->clk_apb)) {
		dev_err(dev, "failed to enable apb clock\n");
		clk_disable_unprepare(priv->clk_iis);
		return -EINVAL;
	}

	/* DMA configuration for TX FIFO */
	priv->playback_dma_data.addr = res->start + SUNXI_I2S_TXFIFO;
	priv->playback_dma_data.maxburst = 4;
	priv->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;

	/* DMA configuration for RX FIFO */
	priv->capture_dma_data.addr = res->start + SUNXI_I2S_RXFIFO;
	priv->capture_dma_data.maxburst = 4;
	priv->capture_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;

	ret = sunxi_i2s_mclk_init(pdev, priv, base);
	if (ret)
		return ret;

	dev_set_drvdata(&pdev->dev, priv);

	ret = devm_snd_soc_register_component(dev, &sunxi_i2s_component, &sunxi_i2s_dai, 1);
	if (ret)
		return ret;

	ret = devm_snd_dmaengine_pcm_register(dev, NULL, 0);
	if (ret)
		return ret;

	/* master hardware enable */
	ret = regmap_update_bits(priv->regmap, SUNXI_I2S_CTL, SUNXI_I2SCTL_GEN_MASK, SUNXI_I2SCTL_GEN);
	if (ret)
		return ret;

	printk("JDS: sunxi_i2s_probe finished\n");
	return 0;

}

static int sunxi_i2s_remove(struct platform_device *pdev)
{
	struct sunxi_priv *priv = platform_get_drvdata(pdev);

	clk_disable_unprepare(priv->clk_apb);
	clk_disable_unprepare(priv->clk_iis);

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
MODULE_AUTHOR("Jon Smirl <jonsmirl@gmail.com>");
MODULE_LICENSE("GPL");
