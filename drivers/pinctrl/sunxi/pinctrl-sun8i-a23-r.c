/*
 * Allwinner A23 SoCs special pins pinctrl driver.
 *
 * Copyright (C) 2014 Chen-Yu Tsai
 * Chen-Yu Tsai <wens@csie.org>
 *
 * Copyright (C) 2014 Boris Brezillon
 * Boris Brezillon <boris.brezillon@free-electrons.com>
 *
 * Copyright (C) 2014 Maxime Ripard
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/reset.h>

#include "pinctrl-sunxi.h"

static const struct sunxi_desc_pin sun8i_a23_r_pins[] = {
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 0),
		  SUNXI_FUNCTION(0x0, "gpio_in"),
		  SUNXI_FUNCTION(0x1, "gpio_out"),
		  SUNXI_FUNCTION(0x2, "s_rsb"),		/* SCK */
		  SUNXI_FUNCTION(0x3, "s_twi")),	/* SCK */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 1),
		  SUNXI_FUNCTION(0x0, "gpio_in"),
		  SUNXI_FUNCTION(0x1, "gpio_out"),
		  SUNXI_FUNCTION(0x2, "s_rsb"),		/* SDA */
		  SUNXI_FUNCTION(0x3, "s_twi")),	/* SDA */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 2),
		  SUNXI_FUNCTION(0x0, "gpio_in"),
		  SUNXI_FUNCTION(0x1, "gpio_out"),
		  SUNXI_FUNCTION(0x2, "s_uart")),	/* TX */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 3),
		  SUNXI_FUNCTION(0x0, "gpio_in"),
		  SUNXI_FUNCTION(0x1, "gpio_out"),
		  SUNXI_FUNCTION(0x2, "s_uart")),	/* RX */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 4),
		  SUNXI_FUNCTION(0x0, "gpio_in"),
		  SUNXI_FUNCTION(0x1, "gpio_out"),
		  SUNXI_FUNCTION(0x3, "s_jtag")),	/* MS */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 5),
		  SUNXI_FUNCTION(0x0, "gpio_in"),
		  SUNXI_FUNCTION(0x1, "gpio_out"),
		  SUNXI_FUNCTION(0x3, "s_jtag")),	/* CK */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 6),
		  SUNXI_FUNCTION(0x0, "gpio_in"),
		  SUNXI_FUNCTION(0x1, "gpio_out"),
		  SUNXI_FUNCTION(0x3, "s_jtag")),	/* DO */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 7),
		  SUNXI_FUNCTION(0x0, "gpio_in"),
		  SUNXI_FUNCTION(0x1, "gpio_out"),
		  SUNXI_FUNCTION(0x3, "s_jtag")),	/* DI */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 8),
		  SUNXI_FUNCTION(0x0, "gpio_in"),
		  SUNXI_FUNCTION(0x1, "gpio_out"),
		  SUNXI_FUNCTION(0x2, "s_twi")),	/* SCK */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 9),
		  SUNXI_FUNCTION(0x0, "gpio_in"),
		  SUNXI_FUNCTION(0x1, "gpio_out"),
		  SUNXI_FUNCTION(0x2, "s_twi")),	/* SDA */
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 10),
		  SUNXI_FUNCTION(0x0, "gpio_in"),
		  SUNXI_FUNCTION(0x1, "gpio_out"),
		  SUNXI_FUNCTION(0x2, "s_pwm")),
	SUNXI_PIN(SUNXI_PINCTRL_PIN(L, 11),
		  SUNXI_FUNCTION(0x0, "gpio_in"),
		  SUNXI_FUNCTION(0x1, "gpio_out")),
};

static const struct sunxi_pinctrl_desc sun8i_a23_r_pinctrl_data = {
	.pins = sun8i_a23_r_pins,
	.npins = ARRAY_SIZE(sun8i_a23_r_pins),
	.pin_base = PL_BASE,
};

static int sun8i_a23_r_pinctrl_probe(struct platform_device *pdev)
{
	struct reset_control *rstc;
	int ret;

	rstc = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(rstc)) {
		dev_err(&pdev->dev, "Reset controller missing\n");
		return PTR_ERR(rstc);
	}

	ret = reset_control_deassert(rstc);
	if (ret)
		return ret;

	ret = sunxi_pinctrl_init(pdev,
				 &sun8i_a23_r_pinctrl_data);

	if (ret)
		reset_control_assert(rstc);

	return ret;
}

static struct of_device_id sun8i_a23_r_pinctrl_match[] = {
	{ .compatible = "allwinner,sun8i-a23-r-pinctrl", },
	{}
};
MODULE_DEVICE_TABLE(of, sun8i_a23_r_pinctrl_match);

static struct platform_driver sun8i_a23_r_pinctrl_driver = {
	.probe	= sun8i_a23_r_pinctrl_probe,
	.driver	= {
		.name		= "sun8i-a23-r-pinctrl",
		.owner		= THIS_MODULE,
		.of_match_table	= sun8i_a23_r_pinctrl_match,
	},
};
module_platform_driver(sun8i_a23_r_pinctrl_driver);

MODULE_AUTHOR("Chen-Yu Tsai <wens@csie.org>");
MODULE_AUTHOR("Boris Brezillon <boris.brezillon@free-electrons.com");
MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com");
MODULE_DESCRIPTION("Allwinner A23 R_PIO pinctrl driver");
MODULE_LICENSE("GPL");
