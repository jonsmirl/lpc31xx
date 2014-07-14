/*
 * Copyright 2013 Emilio López
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

#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

struct sun4i_pll2_clk {
	struct clk_hw hw;
	void __iomem *reg;
};

static inline struct sun4i_pll2_clk *sun4i_pll2_clk(struct clk_hw *hw)
{
	return container_of(hw, struct sun4i_pll2_clk, hw);
}

static unsigned long sun4i_pll2_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct sun4i_pll2_clk *clk = sun4i_pll2_clk(hw);
	int n, prediv, postdiv;

	u32 val = readl(clk->reg);
	n = (val >> 8) & 0x7F;
	prediv = (val >> 0) & 0x1F;
	postdiv = (val >> 26) & 0xF;

	return ((parent_rate * n) / prediv) / postdiv;
}

static long sun4i_pll2_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *parent_rate)
{
	return rate;
}

static int sun4i_pll2_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct sun4i_pll2_clk *clk = sun4i_pll2_clk(hw);
	u32 val;

	if (rate == 22579200) {
		val = BIT(31) | (79 << 8) | (4 << 26) | (21);
	} else  {
		val = BIT(31) | (86 << 8) | (4 << 26) | (21);
	}

	writel(val, clk->reg);

	return 0;
}

static struct clk_ops sun4i_pll2_ops = {
	.recalc_rate = sun4i_pll2_recalc_rate,
	.round_rate = sun4i_pll2_round_rate,
	.set_rate = sun4i_pll2_set_rate,
};

static void __init sun4i_pll2_setup(struct device_node *np)
{
	const char *clk_name = np->name, *parent;
	struct clk_init_data init;
	struct sun4i_pll2_clk *pll2;
	struct clk *clk;

	pll2 = kzalloc(sizeof(*pll2), GFP_KERNEL);
	if (!pll2)
		return;

	pll2->reg = of_iomap(np, 0);
	parent = of_clk_get_parent_name(np, 0);
	of_property_read_string(np, "clock-output-names", &clk_name);

	init.name = clk_name;
	init.ops = &sun4i_pll2_ops;
	init.parent_names = &parent;
	init.num_parents = 1;
	init.flags = 0;

	pll2->hw.init = &init;

	clk = clk_register(NULL, &pll2->hw);
	if (IS_ERR(clk))
		return;

	of_clk_add_provider(np, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(sun4i_pll2, "allwinner,sun4i-a10-pll2-clk", sun4i_pll2_setup);
