/*
 * sunxi-ss.c - hardware cryptographic accelerator for Allwinner A20 SoC
 *
 * Copyright (C) 2013-2014 Corentin LABBE <clabbe.montjoie@gmail.com>
 *
 * Support AES cipher with 128,192,256 bits keysize.
 * Support MD5 and SHA1 hash algorithms.
 * Support DES and 3DES
 * Support PRNG
 *
 * You could find the datasheet at
 * http://dl.linux-sunxi.org/A20/A20%20User%20Manual%202013-03-22.pdf
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2 of the License
 */
#include "sunxi-ss.h"

#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <crypto/scatterwalk.h>
#include <linux/scatterlist.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

/* General notes:
 * I cannot use a key/IV cache because each time one of these change ALL stuff
 * need to be re-writed (rewrite SS_KEYX ans SS_IVX).
 * And for example, with dm-crypt IV changes on each request.
 *
 * After each request the device must be disabled with a write of 0 in SS_CTL
 *
 * For performance reason, we use writel_relaxed/read_relaxed for all
 * operations on RX and TX FIFO and also SS_FCSR.
 * For all other registers, we use writel/readl.
 * See http://permalink.gmane.org/gmane.linux.ports.arm.kernel/117644
 * and http://permalink.gmane.org/gmane.linux.ports.arm.kernel/117640
 * */

static int ss_is_init;
struct sunxi_ss_ctx *ss;
EXPORT_SYMBOL_GPL(ss);

static int sunxi_ss_probe(struct platform_device *pdev)
{
	struct resource *res;
	u32 v;
	int err;
	unsigned long cr;
	const unsigned long cr_ahb = 24 * 1000 * 1000;
	const unsigned long cr_mod = 150 * 1000 * 1000;

	if (!pdev->dev.of_node)
		return -ENODEV;

	if (ss_is_init == 1) {
		dev_err(&pdev->dev, "Device already initialized\n");
		return -ENODEV;
	}

	ss = devm_kzalloc(&pdev->dev, sizeof(*ss), GFP_KERNEL);
	if (ss == NULL)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ss->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ss->base)) {
		dev_err(&pdev->dev, "Cannot request MMIO\n");
		return PTR_ERR(ss->base);
	}

	ss->ssclk = devm_clk_get(&pdev->dev, "mod");
	if (IS_ERR(ss->ssclk)) {
		err = PTR_ERR(ss->ssclk);
		dev_err(&pdev->dev, "Cannot get SS clock err=%d\n", err);
		return err;
	}
	dev_dbg(&pdev->dev, "clock ss acquired\n");

	ss->busclk = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(ss->busclk)) {
		err = PTR_ERR(ss->busclk);
		dev_err(&pdev->dev, "Cannot get AHB SS clock err=%d\n", err);
		return err;
	}
	dev_dbg(&pdev->dev, "clock ahb_ss acquired\n");

	/* Enable the clocks */
	err = clk_prepare_enable(ss->busclk);
	if (err != 0) {
		dev_err(&pdev->dev, "Cannot prepare_enable busclk\n");
		return err;
	}
	err = clk_prepare_enable(ss->ssclk);
	if (err != 0) {
		dev_err(&pdev->dev, "Cannot prepare_enable ssclk\n");
		clk_disable_unprepare(ss->busclk);
		return err;
	}

	/* Check that clock have the correct rates gived in the datasheet */
	/* Try to set the clock to the maximum allowed */
	err = clk_set_rate(ss->ssclk, cr_mod);
	if (err != 0) {
		dev_err(&pdev->dev, "Cannot set clock rate to ssclk\n");
		clk_disable_unprepare(ss->ssclk);
		clk_disable_unprepare(ss->busclk);
		return err;
	}
	cr = clk_get_rate(ss->busclk);
	if (cr >= cr_ahb)
		dev_dbg(&pdev->dev, "Clock bus %lu (%lu MHz) (must be >= %lu)\n",
				cr, cr / 1000000, cr_ahb);
	else
		dev_warn(&pdev->dev, "Clock bus %lu (%lu MHz) (must be >= %lu)\n",
				cr, cr / 1000000, cr_ahb);
	cr = clk_get_rate(ss->ssclk);
	if (cr == cr_mod)
		dev_dbg(&pdev->dev, "Clock ss %lu (%lu MHz) (must be <= %lu)\n",
				cr, cr / 1000000, cr_mod);
	else {
		dev_warn(&pdev->dev, "Clock ss is at %lu (%lu MHz) (must be <= %lu)\n",
				cr, cr / 1000000, cr_mod);
	}

	/* TODO Does this information could be usefull ? */
	writel(SS_ENABLED, ss->base + SS_CTL);
	v = readl(ss->base + SS_CTL);
	v >>= 16;
	v &= 0x07;
	dev_info(&pdev->dev, "Die ID %d\n", v);
	writel(0, ss->base + SS_CTL);

	ss->dev = &pdev->dev;

	mutex_init(&ss->lock);
	mutex_init(&ss->bufin_lock);
	mutex_init(&ss->bufout_lock);
	ss_is_init = 1;
	return 0;
}

/* No need to check is some sub module is loaded,
 * since they need the ss structure symbol */
static int __exit sunxi_ss_remove(struct platform_device *pdev)
{
	if (!pdev->dev.of_node)
		return 0;

	if (ss->buf_in != NULL)
		kfree(ss->buf_in);
	if (ss->buf_out != NULL)
		kfree(ss->buf_out);

	writel(0, ss->base + SS_CTL);
	clk_disable_unprepare(ss->busclk);
	clk_disable_unprepare(ss->ssclk);
	ss_is_init = 0;
	return 0;
}

/*============================================================================*/
/*============================================================================*/
static const struct of_device_id a20ss_crypto_of_match_table[] = {
	{ .compatible = "allwinner,sun7i-a20-crypto" },
	{}
};
MODULE_DEVICE_TABLE(of, a20ss_crypto_of_match_table);

static struct platform_driver sunxi_ss_driver = {
	.probe          = sunxi_ss_probe,
	.remove         = __exit_p(sunxi_ss_remove),
	.driver         = {
		.owner          = THIS_MODULE,
		.name           = "sunxi-ss",
		.of_match_table	= a20ss_crypto_of_match_table,
	},
};

module_platform_driver(sunxi_ss_driver);

MODULE_DESCRIPTION("Allwinner Security System cryptographic accelerator");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Corentin LABBE <clabbe.montjoie@gmail.com>");
