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
 * TODO:
 * - Implement duty_percent=0 to set pwm line to 0 - right now it goes to 100%
 * -
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

#include <linux/module.h>
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
#define SUNXI_PWM_CYCLES_TOTAL_MASK	(0xFFFF<<16)
#define SUNXI_PWM_CYCLES_TOTAL_SHIFT	16
#define SUNXI_PWM_CYCLES_TOTAL_WIDTH	16
#define SUNXI_PWM_CYCLES_ACTIVE_MASK	(0xFFFF<<0)
#define SUNXI_PWM_CYCLES_ACTIVE_SHIFT	0
#define SUNXI_PWM_CYCLES_ACTIVE_WIDTH	16

#define MAX_CYCLES 0x0ffff /* max cycle count possible for period active and entire */

/* Supported SoC families - used for quirks */
enum sunxi_soc_family {
	SUN4I,	/* A10 SoC - later revisions */
	SUN5I,	/* A10S/A13 SoCs */
	SUN7I,	/* A20 SoC */
};

/*
 * structure that defines the pwm control register
 */

enum sun4i_pwm_prescale {
	PRESCALE_DIV120  = 0x00,  /* Divide 24mhz clock by 120 */
	PRESCALE_DIV180  = 0x01,
	PRESCALE_DIV240  = 0x02,
	PRESCALE_DIV360  = 0x03,
	PRESCALE_DIV480  = 0x04,
	PRESCALE_INVx05  = 0x05,
	PRESCALE_INVx06  = 0x06,
	PRESCALE_INVx07  = 0x07,
	PRESCALE_DIV12k  = 0x08,
	PRESCALE_DIV24k  = 0x09,
	PRESCALE_DIV36k  = 0x0a,
	PRESCALE_DIV48k  = 0x0b,
	PRESCALE_DIV72k  = 0x0c
};

static const unsigned int prescale_divisor[] = {120,
						  180,
						  240,
						  360,
						  480,
						  480, /* Invalid Option */
						  480, /* Invalid Option */
						  480, /* Invalid Option */
						  12000,
						  24000,
						  36000,
						  48000,
						  72000};


struct sunxi_pwm_chip {
	struct pwm_chip chip;
	struct clk *clk, *clk_pwm[SUNXI_PWM_CHANNEL_MAX];
	struct clk_divider pwmclk_div;
	struct clk_onecell_data clk_data;
	struct regmap *regmap;
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
enum sun4i_pwm_prescale pwm_get_best_prescale(unsigned long long period_in) 
{
	int i;
	unsigned long period = period_in;
	const unsigned long min_optimal_period_cycles = MAX_CYCLES / 2;
	const unsigned long min_period_cycles = 0x02;
	enum sun4i_pwm_prescale best_prescale = 0;

	best_prescale = -1;
	for(i = 0 ; i < ARRAY_SIZE(prescale_divisor) ; i++) {
		unsigned long int check_value = (prescale_divisor[i] / 24);
		if(check_value < 1 || check_value > period) {
			break;
		}
		if(((period / check_value) >= min_optimal_period_cycles) &&
			((period / check_value) <= MAX_CYCLES)) {
			best_prescale = i;
			break;
		}
	}

	if(best_prescale > ARRAY_SIZE(prescale_divisor)) {
		for(i = 0 ; i < ARRAY_SIZE(prescale_divisor) ; i++) {
			unsigned long int check_value = (prescale_divisor[i] / 24);
			if(check_value < 1 || check_value > period) {
				break;
			}
			if(((period / check_value) >= min_period_cycles) &&
				((period / check_value) <= MAX_CYCLES)) {
				best_prescale = i;
				break;
			}
		}
	}
	if(best_prescale > ARRAY_SIZE(prescale_divisor)) {
		best_prescale = PRESCALE_DIV480;  /* Something that's not zero - use invalid prescale value */
	}

	return best_prescale;
}

/*
 * return the number of cycles for the channel period computed from the microseconds
 * for the period.  Allwinner docs call this "entire" cycles
 */
unsigned int get_entire_cycles(struct pwm_chip *chip, unsigned int prescale, unsigned int period) 
{
	unsigned int entire_cycles = 0x01;

	if ((2 * prescale_divisor[prescale] * MAX_CYCLES) > 0) {
		entire_cycles = period / (prescale_divisor[prescale] / 24);
	}
	if (entire_cycles == 0) {
		entire_cycles = MAX_CYCLES;
	}
	if (entire_cycles > MAX_CYCLES) {
		entire_cycles = MAX_CYCLES;
	}
	dev_dbg(chip->dev, "Best prescale was %d, entire cycles was %u\n", prescale, entire_cycles);

	return entire_cycles;
}

/*
 * return the number of cycles for the channel duty computed from the microseconds
 * for the duty.  Allwinner docs call this "active" cycles
 */
unsigned int get_active_cycles(struct pwm_chip *chip, unsigned int entire_cycles, 
		unsigned int prescale, unsigned int period) 
{
	unsigned int active_cycles = 0x01;

	if ((2 * prescale_divisor[prescale] * MAX_CYCLES) > 0) {
		active_cycles = period / (prescale_divisor[prescale] / 24);
	}
	dev_dbg(chip->dev, "Best prescale was %d, active cycles was %u (before entire check)\n", prescale, active_cycles);

	if(active_cycles > MAX_CYCLES) {
		active_cycles = entire_cycles - 1;
	}
	dev_dbg(chip->dev, "Best prescale was %d, active cycles was %u (after  entire check)\n", prescale, active_cycles);

	return active_cycles;
}

static int sunxi_pwm_busy(struct sunxi_pwm_chip *priv)
{
	int i, reg_val;

	for (i = 0; i < 50; i++) {
		regmap_read(priv->regmap, SUNXI_PWM_CTRL_REG, &reg_val);
		if ((reg_val & (SUNXI_PWMCTL_PWM1_NOTRDY | SUNXI_PWMCTL_PWM0_NOTRDY)) == 0)
			return 0;
		mdelay(1);
	}
	return -EIO;
}


static int sunxi_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			  int duty_ns, int period_ns)
{
	struct sunxi_pwm_chip *priv = to_sunxi_chip(chip);
	unsigned int period, prescale, duty, entire_cycles, active_cycles, reg_val;
	int ret;

	period = period_ns / 1000;
	prescale = pwm_get_best_prescale(period);
	duty = duty_ns / 1000;

	entire_cycles = get_entire_cycles(chip, prescale, period);
	active_cycles = get_active_cycles(chip, entire_cycles, prescale, period);
	if(entire_cycles >= active_cycles && active_cycles) {
		entire_cycles = MAX_CYCLES;
		active_cycles = MAX_CYCLES;
	}

	reg_val = (entire_cycles << SUNXI_PWM_CYCLES_ACTIVE_SHIFT) & SUNXI_PWM_CYCLES_ACTIVE_MASK;
	reg_val = (active_cycles << SUNXI_PWM_CYCLES_ACTIVE_SHIFT) & SUNXI_PWM_CYCLES_ACTIVE_MASK;

	switch (pwm->hwpwm) {
	case 0:
		regmap_update_bits(priv->regmap, SUNXI_PWM_CTRL_REG, SUNXI_PWMCTL_PWM0_PRE_MASK, prescale);
		regmap_write(priv->regmap, SUNXI_PWM_CH0_PERIOD, reg_val);
		break;
	case 1:
		regmap_update_bits(priv->regmap, SUNXI_PWM_CTRL_REG, SUNXI_PWMCTL_PWM1_PRE_MASK, prescale);
		regmap_write(priv->regmap, SUNXI_PWM_CH1_PERIOD, reg_val);
		break;
	default:
		return -EINVAL;
	}

	ret = sunxi_pwm_busy(priv);
	if (ret)
		return ret;

	return 0;
}

static int sunxi_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct sunxi_pwm_chip *priv = to_sunxi_chip(chip);
	int ret;
	
	ret = sunxi_pwm_busy(priv);
	if (ret)
		return ret;

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
	int ret;

	ret = sunxi_pwm_busy(priv);
	if (ret)
		return;

	switch (pwm->hwpwm) {
	case 0:
		regmap_update_bits(priv->regmap, SUNXI_PWM_CTRL_REG,
			SUNXI_PWMCTL_PWM0_GATE_MASK | SUNXI_PWMCTL_PWM0_EN_MASK, 0);
		break;
	case 1:
		regmap_update_bits(priv->regmap, SUNXI_PWM_CTRL_REG,
			SUNXI_PWMCTL_PWM1_GATE_MASK | SUNXI_PWMCTL_PWM1_EN_MASK, 0);
		break;
	default:
		return;
	}
	return;
}

static int sunxi_pwm_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
			       enum pwm_polarity polarity)
{
	struct sunxi_pwm_chip *priv = to_sunxi_chip(chip);
	int ret;

	ret = sunxi_pwm_busy(priv);
	if (ret)
		return ret;

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

static int sunxi_pwm_clk_init(struct platform_device *pdev, struct sunxi_pwm_chip *priv, void __iomem *base)
{
	struct device_node *np = pdev->dev.of_node;
	struct clk_init_data init;
	const char *clk_name = NULL;
	const char *clk_parent = __clk_get_name(priv->clk);
	int i, ret = 0;
	int flags = 0;

	init.ops = &clk_divider_ops;
	init.flags = flags | CLK_IS_BASIC;
	init.parent_names = &clk_parent;
	init.num_parents = 1;

	/* struct clk_divider assignments */
	priv->pwmclk_div.reg = base;
	priv->pwmclk_div.shift = 0;
	priv->pwmclk_div.width = 0;
	priv->pwmclk_div.flags = 0;
	priv->pwmclk_div.lock = NULL;
	priv->pwmclk_div.hw.init = &init;
	priv->pwmclk_div.table = NULL;

	ret = of_property_read_string_index(np, "clock-output-names", 0, &clk_name);
	if (ret)
		return ret;

	printk("JDS - sunxi_pwm_pwmclk_init %s\n", clk_name);

	init.name = clk_name;

	/* register the clock */
	priv->clk_pwm[0] = clk_register(&pdev->dev, &priv->pwmclk_div.hw);
	if (IS_ERR(priv->clk_pwm[i])) {
		dev_err(&pdev->dev, "failed to register pwmclk: %ld\n", PTR_ERR(priv->clk_pwm[i]));
		return PTR_ERR(priv->clk_pwm[i]);
	}

	ret = of_property_read_string_index(np, "clock-output-names", 1, &clk_name);
	if (ret)
		return ret;

	printk("JDS - sunxi_pwm_pwmclk_init %s\n", clk_name);

	init.name = clk_name;

	/* register the clock */
	priv->clk_pwm[1] = clk_register(&pdev->dev, &priv->pwmclk_div.hw);
	if (IS_ERR(priv->clk_pwm[i])) {
		dev_err(&pdev->dev, "failed to register pwmclk: %ld\n", PTR_ERR(priv->clk_pwm[i]));
		return PTR_ERR(priv->clk_pwm[i]);
	}

	priv->clk_data.clks = priv->clk_pwm;
	priv->clk_data.clk_num = 2;
	ret = of_clk_add_provider(np, of_clk_src_onecell_get, &priv->clk_data);
	if (ret)
		return ret;
	return 0;
}

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
};

static int sunxi_pwm_probe(struct platform_device *pdev)
{
	void __iomem *base;
	struct sunxi_pwm_chip *priv;
	struct resource *res;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->chip.dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev, "failed to get Osc24M clock\n");
		return PTR_ERR(priv->clk);
	}

	priv->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					     &sunxi_pwm_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->chip.ops = &sunxi_pwm_ops;
	priv->chip.base = -1;
	priv->chip.npwm = 2;
	priv->chip.of_xlate = of_pwm_xlate_with_flags;
	priv->chip.of_pwm_n_cells = 3;

	ret = pwmchip_add(&priv->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add PWM chip: %d\n", ret);
		return ret;
	}

	ret = sunxi_pwm_clk_init(pdev, priv, base);
	if (ret) {
		dev_err(&pdev->dev, "failed to create PWM clocks: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, priv);

	printk("JDS pwm initialized\n");
	return 0;
}

static int sunxi_pwm_remove(struct platform_device *pdev)
{
	struct sunxi_pwm_chip *priv = platform_get_drvdata(pdev);

	return pwmchip_remove(&priv->chip);
}

static const struct of_device_id sunxi_pwm_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-pwm", .data = (void *)SUN4I},
	{ .compatible = "allwinner,sun5i-a13-pwm", .data = (void *)SUN5I},
	{ .compatible = "allwinner,sun7i-a20-pwm", .data = (void *)SUN7I},
	{}
};
MODULE_DEVICE_TABLE(of, sunxi_pwm_of_match);

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


