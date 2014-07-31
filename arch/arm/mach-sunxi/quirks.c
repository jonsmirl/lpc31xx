/*
 * Runtime quirk handling for sunxi SoCs
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

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "sunxi-soc-id.h"

static int __init update_compatible_string(const char *oldc, const char *newc)
{
	int count = 0;
	struct property *newprop;
	size_t newlen = strlen(newc);
	struct device_node *np = NULL;

	for_each_compatible_node(np, NULL, oldc) {
		newprop = kzalloc(sizeof(*newprop), GFP_KERNEL);
		if (!newprop)
			return -ENOMEM;

		newprop->name = kstrdup("compatible", GFP_KERNEL);
		newprop->value = kstrdup(newc, GFP_KERNEL);
		newprop->length = newlen;

		if (!newprop->name || !newprop->value) {
			kfree(newprop);
			return -ENOMEM;
		}

		of_update_property(np, newprop);
		count++;
	}

	return count;
}

static void __init sun4i_pll2_quirk(void)
{
	/* Only revision A is affected */
	if (sunxi_soc_revision() != 'A')
		return;

	WARN_ON(!update_compatible_string("allwinner,sun4i-a10-b-pll2",
					  "allwinner,sun4i-a10-a-pll2"));
}

static void __init sun4i_codec_quirk(void)
{
	/* Only revision A is affected */
	if (sunxi_soc_revision() != 'A')
		return;

	WARN_ON(!update_compatible_string("allwinner,sun4i-a10-b-codec",
					  "allwinner,sun4i-a10-a-codec"));
}

static int __init sunxi_apply_quirks(void)
{
	if (of_machine_is_compatible("allwinner,sun4i-a10")) {
		sun4i_pll2_quirk();
		sun4i_codec_quirk();
	}

	return 0;
}
postcore_initcall(sunxi_apply_quirks)
