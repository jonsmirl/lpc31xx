/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * Author: Ola Lilja <ola.o.lilja@stericsson.com>,
 *         Roger Nilsson <roger.xr.nilsson@stericsson.com>
 *         for ST-Ericsson.
 *
 * License terms:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include <asm/page.h>

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/slab.h>
#include <linux/platform_data/dma-ste-dma40.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#include "sunxi-codec.h"

#define SUNXI_PLATFORM_PERIODS_BYTES_MIN	128
#define SUNXI_PLATFORM_PERIODS_BYTES_MAX	(64 * PAGE_SIZE)
#define SUNXI_PLATFORM_PERIODS_MIN		2
#define SUNXI_PLATFORM_PERIODS_MAX		48
#define SUNXI_PLATFORM_BUFFER_BYTES_MAX		(2048 * PAGE_SIZE)

static const struct snd_pcm_hardware sunxi_pcm_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_RESUME |
		SNDRV_PCM_INFO_PAUSE,
	.buffer_bytes_max = SUNXI_PLATFORM_BUFFER_BYTES_MAX,
	.period_bytes_min = SUNXI_PLATFORM_PERIODS_BYTES_MIN,
	.period_bytes_max = SUNXI_PLATFORM_PERIODS_BYTES_MAX,
	.periods_min = SUNXI_PLATFORM_PERIODS_MIN,
	.periods_max = SUNXI_PLATFORM_PERIODS_MAX,
};

static struct dma_chan *sunxi_pcm_request_chan(struct snd_soc_pcm_runtime *rtd,
	struct snd_pcm_substream *substream)
{
	struct snd_soc_dai *dai = rtd->cpu_dai;
	u16 per_data_width, mem_data_width;
//jds	struct stedma40_chan_cfg *dma_cfg;
	struct sunxi_codec_dma_params *dma_params;

	printk("JDS - sunxi_pcm_request_chan\n");

#ifdef JDS
	/* DMA */
	priv->tx_dma_chan = dma_request_slave_channel_reason(&pdev->dev, "tx");
	if (IS_ERR(priv->tx_dma_chan)) {
		dev_err(dev, "Unable to acquire DMA channel TX\n");
		ret = PTR_ERR(priv->tx_dma_chan);
		goto err_free_master;
	}

	dma_sconfig.direction = DMA_MEM_TO_DEV;
	dma_sconfig.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	dma_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	dma_sconfig.dst_addr = priv->codec_phys + SUNXI_DAC_TXDATA;
	dma_sconfig.src_maxburst = 1;
	dma_sconfig.dst_maxburst = 1;

	ret = dmaengine_slave_config(priv->tx_dma_chan, &dma_sconfig);
	if (ret) {
		dev_err(dev, "Unable to configure TX DMA slave\n");
		goto err_tx_dma_release;
	}

	priv->rx_dma_chan = dma_request_slave_channel_reason(&pdev->dev, "rx");
	if (IS_ERR(priv->rx_dma_chan)) {
		dev_err(dev, "Unable to acquire DMA channel RX\n");
		ret = PTR_ERR(priv->rx_dma_chan);
		goto err_tx_dma_release;
	}

	dma_sconfig.direction = DMA_DEV_TO_MEM;
	dma_sconfig.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	dma_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	dma_sconfig.src_addr = priv->codec_phys + SUNXI_ADC_RXDATA;
	dma_sconfig.src_maxburst = 1;
	dma_sconfig.dst_maxburst = 1;

	ret = dmaengine_slave_config(priv->rx_dma_chan, &dma_sconfig);
	if (ret) {
		dev_err(dev, "Unable to configure RX DMA slave\n");
		goto err_rx_dma_release;
	}
#endif
	dma_params = snd_soc_dai_get_dma_data(dai, substream);
//JDS	dma_cfg = dma_params->dma_cfg;

	mem_data_width = DMA_SLAVE_BUSWIDTH_2_BYTES;

	switch (dma_params->data_size) {
	case 32:
		per_data_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	case 16:
		per_data_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case 8:
		per_data_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		break;
	default:
		per_data_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
//JDS		dma_cfg->src_info.data_width = mem_data_width;
//JDS		dma_cfg->dst_info.data_width = per_data_width;
	} else {
//JDS		dma_cfg->src_info.data_width = per_data_width;
//JDS		dma_cfg->dst_info.data_width = mem_data_width;
	}

//JDS	return snd_dmaengine_pcm_request_channel(stedma40_filter, dma_cfg);
	printk("JDS - sunxi_pcm_request_chan ret\n");
	return 0;
}

static int sunxi_pcm_prepare_slave_config(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct dma_slave_config *slave_config)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct card_data *priv = rtd->cpu_dai->dev->platform_data;
	struct snd_dmaengine_dai_dma_data *snd_dma_params;
	struct sunxi_msp_dma_params *ste_dma_params;
	dma_addr_t dma_addr;
	int ret;

	printk("JDS - sunxi_pcm_prepare_slave_config\n");
	ret = snd_hwparams_to_dma_slave_config(substream, params, slave_config);
	if (ret)
		return ret;
	
	slave_config->src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	slave_config->dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	slave_config->src_maxburst = 1;
	slave_config->dst_maxburst = 1;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		slave_config->direction = DMA_MEM_TO_DEV;
		slave_config->dst_addr = priv->codec_phys + SUNXI_DAC_TXDATA;
	} else {
		slave_config->direction = DMA_DEV_TO_MEM;
		slave_config->src_addr = priv->codec_phys + SUNXI_ADC_RXDATA;
	}

	printk("JDS - sunxi_pcm_prepare_slave_config ret\n");
	return 0;
}

static const struct snd_dmaengine_pcm_config sunxi_dmaengine_pcm_config = {
	.pcm_hardware = &sunxi_pcm_hw,
	.compat_request_channel = sunxi_pcm_request_chan,
	.prealloc_buffer_size = 128 * 1024,
	.prepare_slave_config = sunxi_pcm_prepare_slave_config,
};

static const struct snd_dmaengine_pcm_config sunxi_dmaengine_of_pcm_config = {
	.compat_request_channel = sunxi_pcm_request_chan,
	.prepare_slave_config = sunxi_pcm_prepare_slave_config,
};

int devm_sunxi_pcm_platform_register(struct platform_device *pdev)
{
	const struct snd_dmaengine_pcm_config *pcm_config;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	printk("JDS - devm_sunxi_pcm_platform_register\n");
	if (np)
		pcm_config = &sunxi_dmaengine_of_pcm_config;
	else
		pcm_config = &sunxi_dmaengine_pcm_config;

	ret = snd_dmaengine_pcm_register(&pdev->dev, pcm_config,
					 SND_DMAENGINE_PCM_FLAG_NO_RESIDUE);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"%s: ERROR: Failed to register platform '%s' (%d)!\n",
			__func__, pdev->name, ret);
		return ret;
	}

	printk("JDS - devm_sunxi_pcm_platform_register ret\n");
	return 0;
}
EXPORT_SYMBOL_GPL(devm_sunxi_pcm_platform_register);

int sunxi_pcm_unregister_platform(struct platform_device *pdev)
{
	snd_dmaengine_pcm_unregister(&pdev->dev);
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_pcm_unregister_platform);
