/* pwm-sunxi.c
 *
 * pwm module for sun4i (and others) like cubieboard and pcduino
 *
 * (C) Copyright 2013
 * David H. Wilkins  <dwil...@conecuh.com>
 * (C) Copyright 2014
 * Jon Smirl <jonsmirl@gmail.com>
 *
 * CHANGELOG:
 * 8.15.2014 - Jon Smirl
 * - Total rewrite for mainline inclusion
 * 10.08.2013 - Stefan Voit <stefan.voit@voit-consulting.com>
 * - Removed bug that caused the PWM to pause quickly when changing parameters
 * - Dropped debug/dump functions
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.         See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#define DEBUG

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>
#include <linux/delay.h>

/*------------------------------------------------------------*/
/* REGISTER definitions */

#define SUNXI_PWM_CTRL_REG	0x00 /* PWM Control Register */
#define SUNXI_PWM_CH0_PERIOD	0x04 /* PWM Channel 0 Period Register */
#define SUNXI_PWM_CH1_PERIOD	0x08 /* PWM Channel 1 Period Register */

#define SUNXI_PWM_CHANNEL_MAX 2

/* SUNXI_PWM_CTRL_REG	0x00	PWM Control Register */
#define SUNXI_PWMCTL_PWM1_NOTRDY	(1<<29)
#define SUNXI_PWMCTL_PWM1_RDY_MASK	(1<<29)
#define SUNXI_PWMCTL_PWM1_RDY_SHIFT	29
#define SUNXI_PWMCTL_PWM1_RDY_WIDTH	1
#define SUNXI_PWMCTL_PWM0_NOTRDY	(1<<28)
#define SUNXI_PWMCTL_PWM0_RDY_MASK	(1<<28)
#define SUNXI_PWMCTL_PWM0_RDY_SHIFT	28
#define SUNXI_PWMCTL_PWM0_RDY_WIDTH	1
#define SUNXI_PWMCTL_PWM1_BYPASS	(1<<24)
#define SUNXI_PWMCTL_PWM1_BYPASS_MASK	(1<<24)
#define SUNXI_PWMCTL_PWM1_BYPASS_SHIFT	24
#define SUNXI_PWMCTL_PWM1_BYPASS_WIDTH	1
#define SUNXI_PWMCTL_PWM1_START		(1<<23)
#define SUNXI_PWMCTL_PWM1_START_MASK	(1<<23)
#define SUNXI_PWMCTL_PWM1_START_SHIFT	23
#define SUNXI_PWMCTL_PWM1_START_WIDTH	1
#define SUNXI_PWMCTL_PWM1_MODE		(1<<22)
#define SUNXI_PWMCTL_PWM1_MODE_MASK	(1<<22)
#define SUNXI_PWMCTL_PWM1_MODE_SHIFT	22
#define SUNXI_PWMCTL_PWM1_MODE_WIDTH	1
#define SUNXI_PWMCTL_PWM1_GATE		(1<<21)
#define SUNXI_PWMCTL_PWM1_GATE_MASK	(1<<21)
#define SUNXI_PWMCTL_PWM1_GATE_SHIFT	21
#define SUNXI_PWMCTL_PWM1_GATE_WIDTH	1
#define SUNXI_PWMCTL_PWM1_STATE		(1<<20)
#define SUNXI_PWMCTL_PWM1_STATE_MASK	(1<<20)
#define SUNXI_PWMCTL_PWM1_STATE_SHIFT	20
#define SUNXI_PWMCTL_PWM1_STATE_WIDTH	1
#define SUNXI_PWMCTL_PWM1_EN		(1<<19)
#define SUNXI_PWMCTL_PWM1_EN_MASK	(1<<19)
#define SUNXI_PWMCTL_PWM1_EN_SHIFT	19
#define SUNXI_PWMCTL_PWM1_EN_WIDTH	1
#define SUNXI_PWMCTL_PWM1_PRE_MASK	(0xf<<15)
#define SUNXI_PWMCTL_PWM1_PRE_120	(0<<15)
#define SUNXI_PWMCTL_PWM1_PRE_180	(1<<15)
#define SUNXI_PWMCTL_PWM1_PRE_240	(2<<15)
#define SUNXI_PWMCTL_PWM1_PRE_360	(3<<15)
#define SUNXI_PWMCTL_PWM1_PRE_480	(4<<15)
#define SUNXI_PWMCTL_PWM1_PRE_12K	(8<<15)
#define SUNXI_PWMCTL_PWM1_PRE_24K	(9<<15)
#define SUNXI_PWMCTL_PWM1_PRE_36K	(0xa<<15)
#define SUNXI_PWMCTL_PWM1_PRE_48K	(0xb<<15)
#define SUNXI_PWMCTL_PWM1_PRE_72K	(0xc<<15)
#define SUNXI_PWMCTL_PWM1_PRE_1		(0xf<<15)
#define SUNXI_PWMCTL_PWM1_PRE_SHIFT	15
#define SUNXI_PWMCTL_PWM1_PRE_WIDTH	4
#define SUNXI_PWMCTL_PWM0_BYPASS	(1<<9)
#define SUNXI_PWMCTL_PWM0_BYPASS_MASK	(1<<9)
#define SUNXI_PWMCTL_PWM0_BYPASS_SHIFT	9
#define SUNXI_PWMCTL_PWM0_BYPASS_WIDTH	1
#define SUNXI_PWMCTL_PWM0_START		(1<<8)
#define SUNXI_PWMCTL_PWM0_START_MASK	(1<<8)
#define SUNXI_PWMCTL_PWM0_START_SHIFT	8
#define SUNXI_PWMCTL_PWM0_START_WIDTH	1
#define SUNXI_PWMCTL_PWM0_MODE		(1<<7)
#define SUNXI_PWMCTL_PWM0_MODE_MASK	(1<<7)
#define SUNXI_PWMCTL_PWM0_MODE_SHIFT	7
#define SUNXI_PWMCTL_PWM0_MODE_WIDTH	1
#define SUNXI_PWMCTL_PWM0_GATE		(1<<6)
#define SUNXI_PWMCTL_PWM0_GATE_MASK	(1<<6)
#define SUNXI_PWMCTL_PWM0_GATE_SHIFT	6
#define SUNXI_PWMCTL_PWM0_GATE_WIDTH	1
#define SUNXI_PWMCTL_PWM0_STATE		(1<<5)
#define SUNXI_PWMCTL_PWM0_STATE_MASK	(1<<5)
#define SUNXI_PWMCTL_PWM0_STATE_SHIFT	5
#define SUNXI_PWMCTL_PWM0_STATE_WIDTH	1
#define SUNXI_PWMCTL_PWM0_EN		(1<<4)
#define SUNXI_PWMCTL_PWM0_EN_MASK	(1<<4)
#define SUNXI_PWMCTL_PWM0_EN_SHIFT	4
#define SUNXI_PWMCTL_PWM0_EN_WIDTH	1
#define SUNXI_PWMCTL_PWM0_PRE_MASK	(0xf<<0)
#define SUNXI_PWMCTL_PWM0_PRE_120	(0<<0)
#define SUNXI_PWMCTL_PWM0_PRE_180	(1<<0)
#define SUNXI_PWMCTL_PWM0_PRE_240	(2<<0)
#define SUNXI_PWMCTL_PWM0_PRE_360	(3<<0)
#define SUNXI_PWMCTL_PWM0_PRE_480	(4<<0)
#define SUNXI_PWMCTL_PWM0_PRE_12K	(8<<0)
#define SUNXI_PWMCTL_PWM0_PRE_24K	(9<<0)
#define SUNXI_PWMCTL_PWM0_PRE_36K	(0xa<<0)
#define SUNXI_PWMCTL_PWM0_PRE_48K	(0xb<<0)
#define SUNXI_PWMCTL_PWM0_PRE_72K	(0xc<<0)
#define SUNXI_PWMCTL_PWM0_PRE_1		(0xf<<0)
#define SUNXI_PWMCTL_PWM0_PRE_SHIFT	0
#define SUNXI_PWMCTL_PWM0_PRE_WIDTH	4

/* SUNXI_PWM_CH0_PERIOD	0x04	PWM Channel 0 Period Register */
/* SUNXI_PWM_CH1_PERIOD	0x08	PWM Channel 1 Period Register */
#define SUNXI_PWM_CYCLES_TOTAL_MASK	(0xFFFFL<<16)
#define SUNXI_PWM_CYCLES_TOTAL_SHIFT	16
#define SUNXI_PWM_CYCLES_TOTAL_WIDTH	16
#define SUNXI_PWM_CYCLES_ACTIVE_MASK	(0xFFFF<<0)
#define SUNXI_PWM_CYCLES_ACTIVE_SHIFT	0
#define SUNXI_PWM_CYCLES_ACTIVE_WIDTH	16

#define MAX_CYCLES_SUN4I 0x0ffL /* max cycle count possible for period active and entire */
#define MAX_CYCLES 0x0ffffL /* max cycle count possible for period active and entire */
#define OSC24 24L /* 24Mhz system oscillator */

/* Supported SoC families - used for quirks */
enum sunxi_soc_family {
	SUN4I,	/* A10 SoC - later revisions */
	SUN5I,	/* A10S/A13 SoCs */
	SUN7I,	/* A20 SoC */
};

/*
 * structure that defines the pwm control register
 */

static unsigned int prescale_divisor[] = {
	120, 180, 240, 360, 480, 480, 480, 480,
	12000, 24000, 36000, 48000, 72000, 72000, 72000, 1
};

struct sunxi_pwm_chip {
	struct pwm_chip chip;
	struct clk *clk;
	struct regmap *regmap;
	enum sunxi_soc_family revision;
	unsigned long max_cycles;
};

static inline struct sunxi_pwm_chip *to_sunxi_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct sunxi_pwm_chip, chip);
}

/*
 * Find the best prescale value for the period
 * We want to get the highest period cycle count possible, so we look
 * make a run through the prescale values looking for numbers over
 * min_optimal_period_cycles.  If none are found then root though again
 * taking anything that works
 */
int pwm_get_best_prescale(struct sunxi_pwm_chip *priv, int period_in) 
{
	int i;
	unsigned long int clk_pico, period, min_optimal_period_cycles;
	const unsigned long min_period_cycles = 0x02;
	int best_prescale = 0;

	period = period_in * 1000; /* convert to picoseconds */
	min_optimal_period_cycles = priv->max_cycles / 2;

	best_prescale = -1;
	for(i = 0 ; i < ARRAY_SIZE(prescale_divisor) ; i++) {

		clk_pico = 1000000L * prescale_divisor[i] / OSC24;
		if(clk_pico < 1 || clk_pico > period) {
			continue;
		}
		if(((period / clk_pico) >= min_optimal_period_cycles) &&
			((period / clk_pico) <= priv->max_cycles)) {
			best_prescale = i;
		}
	}

	if(best_prescale > ARRAY_SIZE(prescale_divisor)) {
		for(i = 0 ; i < ARRAY_SIZE(prescale_divisor) ; i++) {
			clk_pico = 1000000L * prescale_divisor[i] / OSC24;
			if(clk_pico < 1 || clk_pico > period) {
				continue;
			}
			if(((period / clk_pico) >= min_period_cycles) &&
				((period / clk_pico) <= priv->max_cycles)) {
				best_prescale = i;
			}
		}
	}

	if(best_prescale > ARRAY_SIZE(prescale_divisor))
		return -EINVAL;

	dev_dbg(priv->chip.dev, "Best prescale is %d\n", best_prescale);
	return best_prescale;
}

/*
 * return the number of cycles for the channel period computed from the nanoseconds
 * for the period.  Allwinner docs call this "entire" cycles
 */
unsigned int compute_cycles(struct sunxi_pwm_chip *priv, int prescale, int period) 
{
	unsigned long int clk_pico, cycles;

	clk_pico = 1000000L * prescale_divisor[prescale] / OSC24;
	cycles = DIV_ROUND_CLOSEST(period * 1000L, clk_pico);
	if (cycles > priv->max_cycles)
		cycles = priv->max_cycles;
	if (cycles < 2)
		cycles = 2;

	dev_dbg(priv->chip.dev, "Best prescale was %d, cycles is %lu\n", prescale, cycles);

	return cycles;
}

static int sunxi_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			  int duty_ns, int period_ns)
{
	struct sunxi_pwm_chip *priv = to_sunxi_chip(chip);
	int prescale, entire_cycles, active_cycles;
	unsigned int reg_val;

	
	if ((duty_ns <= 0) || (period_ns <= 0))
		return 0;

	// If period less than two cycles, just enable the OSC24 clock bypass 
	if ((priv->revision != SUN4I) && (period_ns < (2 * 1000 / OSC24 + 1))) {
		switch (pwm->hwpwm) {
		case 0:
			regmap_update_bits(priv->regmap, SUNXI_PWM_CTRL_REG,
					SUNXI_PWMCTL_PWM0_BYPASS_MASK, SUNXI_PWMCTL_PWM0_BYPASS);
			break;
		case 1:
			regmap_update_bits(priv->regmap, SUNXI_PWM_CTRL_REG,
					SUNXI_PWMCTL_PWM1_BYPASS_MASK, SUNXI_PWMCTL_PWM1_BYPASS);
			break;
		}
		return 0;
	}

	prescale = pwm_get_best_prescale(priv, period_ns);
	if (prescale < 0)
		return prescale;

	entire_cycles = compute_cycles(priv, prescale, period_ns);
	active_cycles = compute_cycles(priv, prescale, duty_ns);

	reg_val = (entire_cycles << SUNXI_PWM_CYCLES_TOTAL_SHIFT) & SUNXI_PWM_CYCLES_TOTAL_MASK;
	reg_val |= (active_cycles << SUNXI_PWM_CYCLES_ACTIVE_SHIFT) & SUNXI_PWM_CYCLES_ACTIVE_MASK;

	switch (pwm->hwpwm) {
	case 0:
		regmap_update_bits(priv->regmap, SUNXI_PWM_CTRL_REG,
				SUNXI_PWMCTL_PWM0_BYPASS_MASK | SUNXI_PWMCTL_PWM0_PRE_MASK | SUNXI_PWMCTL_PWM0_EN_MASK,
				prescale << SUNXI_PWMCTL_PWM0_PRE_SHIFT | SUNXI_PWMCTL_PWM0_EN);
		regmap_write(priv->regmap, SUNXI_PWM_CH0_PERIOD, reg_val);
		break;
	case 1:
		regmap_update_bits(priv->regmap, SUNXI_PWM_CTRL_REG,
				SUNXI_PWMCTL_PWM1_BYPASS_MASK | SUNXI_PWMCTL_PWM1_PRE_MASK | SUNXI_PWMCTL_PWM1_EN_MASK,
				prescale << SUNXI_PWMCTL_PWM1_PRE_SHIFT | SUNXI_PWMCTL_PWM1_EN);
		regmap_write(priv->regmap, SUNXI_PWM_CH1_PERIOD, reg_val);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sunxi_pwm_busy(struct sunxi_pwm_chip *priv, int num)
{
	int i, reg_val;

	for (i = 0; i < 50; i++) {
		regmap_read(priv->regmap, SUNXI_PWM_CTRL_REG, &reg_val);
		switch (num) {
		case 0:
			if ((reg_val & SUNXI_PWMCTL_PWM0_NOTRDY) == 0)
				return 0;
			break;
		case 1:
			if ((reg_val & SUNXI_PWMCTL_PWM1_NOTRDY) == 0)
				return 0;
			break;
		default:
			return -EINVAL;
		}
		mdelay(1);
	}
	dev_err(priv->chip.dev, "PWM busy timeout\n");
	return -EBUSY;
}


static int sunxi_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct sunxi_pwm_chip *priv = to_sunxi_chip(chip);
	int ret;
	
	if ((priv->revision == SUN5I) || (priv->revision == SUN7I)) {
		ret = sunxi_pwm_busy(priv, pwm->hwpwm);
		if (ret)
			return ret;
	}
	switch (pwm->hwpwm) {
	case 0:
		regmap_update_bits(priv->regmap, SUNXI_PWM_CTRL_REG,
			SUNXI_PWMCTL_PWM0_GATE_MASK | SUNXI_PWMCTL_PWM0_EN_MASK,
			SUNXI_PWMCTL_PWM0_GATE | SUNXI_PWMCTL_PWM0_EN);
		break;
	case 1:
		regmap_update_bits(priv->regmap, SUNXI_PWM_CTRL_REG,
			SUNXI_PWMCTL_PWM1_GATE_MASK | SUNXI_PWMCTL_PWM1_EN_MASK,
			SUNXI_PWMCTL_PWM1_GATE | SUNXI_PWMCTL_PWM1_EN);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void sunxi_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct sunxi_pwm_chip *priv = to_sunxi_chip(chip);

	switch (pwm->hwpwm) {
	case 0:
		regmap_update_bits(priv->regmap, SUNXI_PWM_CTRL_REG,
			SUNXI_PWMCTL_PWM0_GATE_MASK | SUNXI_PWMCTL_PWM0_EN_MASK, 0);
		break;
	case 1:
		regmap_update_bits(priv->regmap, SUNXI_PWM_CTRL_REG,
			SUNXI_PWMCTL_PWM1_GATE_MASK | SUNXI_PWMCTL_PWM1_EN_MASK, 0);
		break;
	}
	return;
}

static int sunxi_pwm_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
			       enum pwm_polarity polarity)
{
	struct sunxi_pwm_chip *priv = to_sunxi_chip(chip);

	switch (pwm->hwpwm) {
	case 0:
		regmap_update_bits(priv->regmap, SUNXI_PWM_CTRL_REG,
			SUNXI_PWMCTL_PWM0_STATE_MASK,
			(polarity == PWM_POLARITY_INVERSED) << SUNXI_PWMCTL_PWM0_STATE_SHIFT);
		break;
	case 1:
		regmap_update_bits(priv->regmap, SUNXI_PWM_CTRL_REG,
			SUNXI_PWMCTL_PWM1_STATE_MASK,
			(polarity == PWM_POLARITY_INVERSED) << SUNXI_PWMCTL_PWM1_STATE_SHIFT);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct pwm_ops sunxi_pwm_ops = {
	.config = sunxi_pwm_config,
	.enable = sunxi_pwm_enable,
	.disable = sunxi_pwm_disable,
	.set_polarity = sunxi_pwm_polarity,
	.owner = THIS_MODULE,
};

static const struct regmap_range sunxi_pwm_volatile_regs_range[] = {
	regmap_reg_range(SUNXI_PWM_CTRL_REG, SUNXI_PWM_CTRL_REG),
};

static const struct regmap_access_table sunxi_pwm_volatile_regs = {
	.yes_ranges	= sunxi_pwm_volatile_regs_range,
	.n_yes_ranges	= ARRAY_SIZE(sunxi_pwm_volatile_regs_range),
};

static const struct regmap_config sunxi_pwm_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= SUNXI_PWM_CH1_PERIOD,
	.volatile_table	= &sunxi_pwm_volatile_regs,
	.fast_io	= true,
};

static const struct of_device_id sunxi_pwm_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-pwm", .data = (void *)SUN4I},
	{ .compatible = "allwinner,sun5i-a13-pwm", .data = (void *)SUN5I},
	{ .compatible = "allwinner,sun7i-a20-pwm", .data = (void *)SUN7I},
	{}
};
MODULE_DEVICE_TABLE(of, sunxi_pwm_of_match);

static int sunxi_pwm_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	void __iomem *base;
	struct sunxi_pwm_chip *priv;
	struct resource *res;
	int ret;

	of_id = of_match_device(sunxi_pwm_of_match, &pdev->dev);
	if (!of_id)
		return -EINVAL;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->chip.dev = &pdev->dev;
	priv->revision = (enum sunxi_soc_family)of_id->data;
	if (priv->revision == SUN4I) {
		priv->max_cycles = MAX_CYCLES_SUN4I;
		prescale_divisor[15] = 72000; //A10 does not support prescale = 1
	} else
		priv->max_cycles = MAX_CYCLES;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					     &sunxi_pwm_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev, "failed to get Osc24M clock\n");
		return PTR_ERR(priv->clk);
	}
	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable Osc24M clock\n");
		return ret;
	}

	priv->chip.ops = &sunxi_pwm_ops;
	priv->chip.base = -1;
	priv->chip.npwm = 2;
	priv->chip.of_xlate = of_pwm_xlate_with_flags;
	priv->chip.of_pwm_n_cells = 3;

	ret = pwmchip_add(&priv->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add PWM chip: %d\n", ret);
		goto error;
	}

	platform_set_drvdata(pdev, priv);
	return 0;

error:
	clk_disable_unprepare(priv->clk);
	return ret;
}

static int sunxi_pwm_remove(struct platform_device *pdev)
{
	struct sunxi_pwm_chip *priv = platform_get_drvdata(pdev);

	clk_disable_unprepare(priv->clk);
	return pwmchip_remove(&priv->chip);
}

static struct platform_driver sunxi_pwm_driver = {
	.driver = {
		.name = "sunxi-pwm",
		.of_match_table = sunxi_pwm_of_match,
	},
	.probe = sunxi_pwm_probe,
	.remove = sunxi_pwm_remove,
};
module_platform_driver(sunxi_pwm_driver);

MODULE_DESCRIPTION("Allwinner PWM Driver");
MODULE_ALIAS("platform:sunxi-pwm");
MODULE_LICENSE("GPL");


