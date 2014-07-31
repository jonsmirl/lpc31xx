/*
 * SoC revision detection for sunxi SoCs
 *
 * Copyright 2014 Emilio López
 *
 * Emilio López <emilio@elopez.com.ar>
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

#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sys_soc.h>

#include "sunxi-soc-id.h"

/*
 * On the A10 SoC, we can read the revision information from the timer
 * block. The detection logic is extracted from similar code on the
 * Allwinner vendor tree, as this is undocumented on the user manual
 */

#define TIMER_SOC_REV_REG		0x13c
#define TIMER_SOC_REV_CLEAR(val)	((val) & ~(0x3 << 6))
#define TIMER_SOC_REV_GET(val)		(((val) >> 6) & 0x3)

static const struct of_device_id sun4i_timer_compatible[] __initconst = {
	{ .compatible = "allwinner,sun4i-a10-timer", },
	{},
};

static int __init sun4i_get_soc_revision(void)
{
	struct device_node *np;
	void __iomem *base;
	u32 val;
	int ret;

	/* Find the timer node */
	np = of_find_matching_node(NULL, sun4i_timer_compatible);
	if (!np)
		return -ENODEV;

	/* Temporarily map it for reading */
	base = of_iomap(np, 0);
	if (!base) {
		of_node_put(np);
		return -ENOMEM;
	}

	/* Clear the SoC revision bits and rewrite the register */
	val = readl(base + TIMER_SOC_REV_REG);
	val = TIMER_SOC_REV_CLEAR(val);
	writel(val, base + TIMER_SOC_REV_REG);

	/* Now read it again and see what shows up */
	val = readl(base + TIMER_SOC_REV_REG);
	val = TIMER_SOC_REV_GET(val);

	switch (val) {
	case 0:  /* revision A */
		ret = 'A';
	case 3:  /* revision B */
		ret = 'B';
	default: /* revision C */
		ret = 'C';
	}

	iounmap(base);
	of_node_put(np);

	return ret;
}

/*
 * On the sun5i SoCs (A10S, A13), we can read the revision information
 * from the first bits in the Security ID. The detection logic is
 * extracted from similar code on the Allwinner vendor tree, as this
 * is undocumented on the user manual.
 */

static const struct of_device_id sun5i_sid_compatible[] __initconst = {
	{ .compatible = "allwinner,sun4i-a10-sid", },
	{},
};

static int __init sun5i_get_soc_revision(void)
{
	struct device_node *np;
	void __iomem *sid;
	u32 val;
	int ret;

	/* Find the SID node */
	np = of_find_matching_node(NULL, sun5i_sid_compatible);
	if (!np)
		return -ENODEV;

	/* Temporarily map it for reading */
	sid = of_iomap(np, 0);
	if (!sid) {
		of_node_put(np);
		return -ENOMEM;
	}

	/* Read and extract the chip revision from the SID */
	val = readl(sid);
	val = (val >> 8) & 0xffffff;

	switch (val) {
	case 0:        /* A10S/A13 rev A */
	case 0x162541: /* A10S/A13 rev A */
	case 0x162565: /* A13 rev A */
		ret = 'A';
		break;
	case 0x162542: /* A10S/A13 rev B */
		ret = 'B';
		break;
	default:       /* Unknown chip revision */
		ret = -ENODATA;
	}

	iounmap(sid);
	of_node_put(np);

	return ret;
}

int __init sunxi_soc_revision(void)
{
	static int revision = -ENODEV;

	/* Try to query the hardware just once */
	if (!IS_ERR_VALUE(revision))
		return revision;

	if (of_machine_is_compatible("allwinner,sun4i-a10")) {
		revision = sun4i_get_soc_revision();
	} else if (of_machine_is_compatible("allwinner,sun5i-a10s") ||
		   of_machine_is_compatible("allwinner,sun5i-a13")) {
		revision = sun5i_get_soc_revision();
	}

	return revision;
}

/* Matches for the sunxi SoCs we know of */
static const struct of_device_id soc_matches[] __initconst = {
	{ .compatible = "allwinner,sun4i-a10", .data = "A10 (sun4i)" },
	{ .compatible = "allwinner,sun5i-a13", .data = "A13 (sun5i)" },
	{ .compatible = "allwinner,sun5i-a10s", .data = "A10S (sun5i)" },
	{ .compatible = "allwinner,sun6i-a31", .data = "A31 (sun6i)" },
	{ .compatible = "allwinner,sun7i-a20", .data = "A20 (sun7i)" },
	{ .compatible = "allwinner,sun8i-a23", .data = "A23 (sun8i)" },
	{ /* sentinel */ },
};

static int __init sunxi_register_soc_device(void)
{
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;
	const struct of_device_id *match;
	struct device_node *root;
	int revision;

	/* Only run on sunxi SoCs that we know of */
	root = of_find_node_by_path("/");
	match = of_match_node(soc_matches, root);
	if (!match)
		goto exit;

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		goto exit;

	/* Read the machine name if available */
	of_property_read_string(root, "model", &soc_dev_attr->machine);

	soc_dev_attr->family = kstrdup("Allwinner A Series", GFP_KERNEL);
	soc_dev_attr->soc_id = kstrdup(match->data, GFP_KERNEL);

	/* Revision may not always be available */
	revision = sunxi_soc_revision();
	if (IS_ERR_VALUE(revision))
		soc_dev_attr->revision = kstrdup("Unknown", GFP_KERNEL);
	else
		soc_dev_attr->revision = kasprintf(GFP_KERNEL, "%c", revision);

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev))
		goto free_struct;

	/*
	 * Print an informational line mentioning the hardware details.
	 * It may come in handy during bug reports, as some early SoC
	 * revisions have hardware quirks and do not get much testing.
	 */
	pr_info("SoC bus registered, running %s %s, revision %s\n",
		soc_dev_attr->family, soc_dev_attr->soc_id,
		soc_dev_attr->revision);

	return 0;

free_struct:
	kfree(soc_dev_attr->family);
	kfree(soc_dev_attr->soc_id);
	kfree(soc_dev_attr->revision);
	kfree(soc_dev_attr);
exit:
	of_node_put(root);

	return 0;
}
postcore_initcall(sunxi_register_soc_device)
