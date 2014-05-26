/*
 * AXP20x regulators driver.
 *
 * Copyright (C) 2013 Carlo Caione <carlo@caione.org>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/axp20x.h>
#include <linux/mfd/core.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define AXP20X_IO_ENABLED		0x03
#define AXP20X_IO_DISABLED		0x07

#define AXP22X_IO_ENABLED		0x04
#define AXP22X_IO_DISABLED		0x03

#define AXP20X_WORKMODE_DCDC2_MASK	BIT(2)
#define AXP20X_WORKMODE_DCDC3_MASK	BIT(1)

#define AXP20X_FREQ_DCDC_MASK		0x0f

#define AXP_DESC_IO(_family, _id, _supply, _min, _max, _step, _vreg, _vmask,	\
		    _ereg, _emask, _enable_val, _disable_val)			\
	[_family##_##_id] = {							\
		.name		= #_id,						\
		.supply_name	= (_supply),					\
		.type		= REGULATOR_VOLTAGE,				\
		.id		= _family##_##_id,				\
		.n_voltages	= (((_max) - (_min)) / (_step) + 1),		\
		.owner		= THIS_MODULE,					\
		.min_uV		= (_min) * 1000,				\
		.uV_step	= (_step) * 1000,				\
		.vsel_reg	= (_vreg),					\
		.vsel_mask	= (_vmask),					\
		.enable_reg	= (_ereg),					\
		.enable_mask	= (_emask),					\
		.enable_val	= (_enable_val),				\
		.disable_val	= (_disable_val),				\
		.ops		= &axp20x_ops,					\
	}

#define AXP_DESC(_family, _id, _supply, _min, _max, _step, _vreg, _vmask, 	\
		 _ereg, _emask) 						\
	[_family##_##_id] = {							\
		.name		= #_id,						\
		.supply_name	= (_supply),					\
		.type		= REGULATOR_VOLTAGE,				\
		.id		= _family##_##_id,				\
		.n_voltages	= (((_max) - (_min)) / (_step) + 1),		\
		.owner		= THIS_MODULE,					\
		.min_uV		= (_min) * 1000,				\
		.uV_step	= (_step) * 1000,				\
		.vsel_reg	= (_vreg),					\
		.vsel_mask	= (_vmask),					\
		.enable_reg	= (_ereg),					\
		.enable_mask	= (_emask),					\
		.ops		= &axp20x_ops,					\
	}

#define AXP_DESC_FIXED(_family, _id, _supply, _volt)				\
	[_family##_##_id] = {							\
		.name		= #_id,						\
		.supply_name	= (_supply),					\
		.type		= REGULATOR_VOLTAGE,				\
		.id		= _family##_##_id,				\
		.n_voltages	= 1,						\
		.owner		= THIS_MODULE,					\
		.min_uV		= (_volt) * 1000,				\
		.ops		= &axp20x_ops_fixed				\
	}

#define AXP_DESC_TABLE(_family, _id, _supply, _table, _vreg, _vmask, _ereg,	\
		       _emask)							\
	[_family##_##_id] = {							\
		.name		= #_id,						\
		.supply_name	= (_supply),					\
		.type		= REGULATOR_VOLTAGE,				\
		.id		= _family##_##_id,				\
		.n_voltages	= ARRAY_SIZE(_table),				\
		.owner		= THIS_MODULE,					\
		.vsel_reg	= (_vreg),					\
		.vsel_mask	= (_vmask),					\
		.enable_reg	= (_ereg),					\
		.enable_mask	= (_emask),					\
		.volt_table	= (_table),					\
		.ops		= &axp20x_ops_table,				\
	}

static const int axp20x_ldo4_data[] = { 1250000, 1300000, 1400000, 1500000, 1600000,
					1700000, 1800000, 1900000, 2000000, 2500000,
					2700000, 2800000, 3000000, 3100000, 3200000,
					3300000 };

static struct regulator_ops axp20x_ops_fixed = {
	.list_voltage		= regulator_list_voltage_linear,
};

static struct regulator_ops axp20x_ops_table = {
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.list_voltage		= regulator_list_voltage_table,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
};

static struct regulator_ops axp20x_ops = {
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.list_voltage		= regulator_list_voltage_linear,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
};

static const struct regulator_desc axp20x_regulators[] = {
	AXP_DESC(AXP20X, DCDC2, "vin2", 700, 2275, 25, AXP20X_DCDC2_V_OUT, 0x3f,
		    AXP20X_PWR_OUT_CTRL, 0x10),
	AXP_DESC(AXP20X, DCDC3, "vin3", 700, 3500, 25, AXP20X_DCDC3_V_OUT, 0x7f,
		    AXP20X_PWR_OUT_CTRL, 0x02),
	AXP_DESC_FIXED(AXP20X, LDO1, "acin", 1300),
	AXP_DESC(AXP20X, LDO2, "ldo24in", 1800, 3300, 100, AXP20X_LDO24_V_OUT, 0xf0,
		    AXP20X_PWR_OUT_CTRL, 0x04),
	AXP_DESC(AXP20X, LDO3, "ldo3in", 700, 3500, 25, AXP20X_LDO3_V_OUT, 0x7f,
		    AXP20X_PWR_OUT_CTRL, 0x40),
	AXP_DESC_TABLE(AXP20X, LDO4, "ldo24in", axp20x_ldo4_data, AXP20X_LDO24_V_OUT, 0x0f,
			  AXP20X_PWR_OUT_CTRL, 0x08),
	AXP_DESC_IO(AXP20X, LDO5, "ldo5in", 1800, 3300, 100, AXP20X_LDO5_V_OUT, 0xf0,
		       AXP20X_GPIO0_CTRL, 0x07, AXP20X_IO_ENABLED,
		       AXP20X_IO_DISABLED),
};

static const struct regulator_desc axp22x_regulators[] = {
	AXP_DESC(AXP22X, DCDC1, "vin1", 1600, 3400, 100, AXP22X_DCDC1_V_OUT, 0x1f,
		    AXP22X_PWR_OUT_CTRL1, BIT(1)),
	AXP_DESC(AXP22X, DCDC2, "vin2", 600, 1540, 20, AXP22X_DCDC2_V_OUT, 0x3f,
		    AXP22X_PWR_OUT_CTRL1, BIT(2)),
	AXP_DESC(AXP22X, DCDC3, "vin3", 600, 1860, 20, AXP22X_DCDC3_V_OUT, 0x3f,
		    AXP22X_PWR_OUT_CTRL1, BIT(3)),
	AXP_DESC(AXP22X, DCDC4, "vin4", 600, 1540, 20, AXP22X_DCDC4_V_OUT, 0x3f,
		    AXP22X_PWR_OUT_CTRL1, BIT(3)),
	AXP_DESC(AXP22X, DCDC5, "vin5", 1000, 2550, 50, AXP22X_DCDC5_V_OUT, 0x1f,
		    AXP22X_PWR_OUT_CTRL1, BIT(4)),
	AXP_DESC(AXP22X, DC5LDO, "vin5", 700, 1400, 100, AXP22X_DC5LDO_V_OUT, 0x7,
		    AXP22X_PWR_OUT_CTRL1, BIT(0)),
	AXP_DESC(AXP22X, ALDO1, "aldoin", 700, 3300, 100, AXP22X_ALDO1_V_OUT, 0x1f,
		    AXP22X_PWR_OUT_CTRL1, BIT(6)),
	AXP_DESC(AXP22X, ALDO2, "aldoin", 700, 3300, 100, AXP22X_ALDO2_V_OUT, 0x1f,
		    AXP22X_PWR_OUT_CTRL1, BIT(7)),
	AXP_DESC(AXP22X, ALDO3, "aldoin", 700, 3300, 100, AXP22X_ALDO3_V_OUT, 0x1f,
		    AXP22X_PWR_OUT_CTRL3, BIT(7)),
	AXP_DESC(AXP22X, DLDO1, "dldoin", 700, 3300, 100, AXP22X_DLDO1_V_OUT, 0x1f,
		    AXP22X_PWR_OUT_CTRL2, BIT(3)),
	AXP_DESC(AXP22X, DLDO2, "dldoin", 700, 3300, 100, AXP22X_DLDO2_V_OUT, 0x1f,
		    AXP22X_PWR_OUT_CTRL2, BIT(4)),
	AXP_DESC(AXP22X, DLDO3, "dldoin", 700, 3300, 100, AXP22X_DLDO3_V_OUT, 0x1f,
		    AXP22X_PWR_OUT_CTRL2, BIT(5)),
	AXP_DESC(AXP22X, DLDO4, "dldoin", 700, 3300, 100, AXP22X_DLDO4_V_OUT, 0x1f,
		    AXP22X_PWR_OUT_CTRL2, BIT(6)),
	AXP_DESC(AXP22X, ELDO1, "eldoin", 700, 3300, 100, AXP22X_ELDO1_V_OUT, 0x1f,
		    AXP22X_PWR_OUT_CTRL2, BIT(0)),
	AXP_DESC(AXP22X, ELDO2, "eldoin", 700, 3300, 100, AXP22X_ELDO2_V_OUT, 0x1f,
		    AXP22X_PWR_OUT_CTRL2, BIT(1)),
	AXP_DESC(AXP22X, ELDO3, "eldoin", 700, 3300, 100, AXP22X_ELDO3_V_OUT, 0x1f,
		    AXP22X_PWR_OUT_CTRL2, BIT(2)),
	AXP_DESC_IO(AXP22X, LDO_IO0, "ldoioin", 1800, 3300, 100, AXP22X_LDO_IO0_V_OUT,
		       0x1f, AXP20X_GPIO0_CTRL, 0x07, AXP22X_IO_ENABLED,
		       AXP22X_IO_DISABLED),
	AXP_DESC_IO(AXP22X, LDO_IO1, "ldoioin", 1800, 3300, 100, AXP22X_LDO_IO1_V_OUT,
		       0x1f, AXP20X_GPIO1_CTRL, 0x07, AXP22X_IO_ENABLED,
		       AXP22X_IO_DISABLED),
	AXP_DESC_FIXED(AXP22X, RTC_LDO, "rtcldoin", 3000),
};

#define AXP_MATCH(_family, _name, _id) \
	[_family##_##_id] = { \
		.name		= #_name, \
		.driver_data	= (void *) &axp20x_regulators[_family##_##_id], \
	}

static struct of_regulator_match axp20x_matches[] = {
	AXP_MATCH(AXP20X, dcdc2, DCDC2),
	AXP_MATCH(AXP20X, dcdc3, DCDC3),
	AXP_MATCH(AXP20X, ldo1, LDO1),
	AXP_MATCH(AXP20X, ldo2, LDO2),
	AXP_MATCH(AXP20X, ldo3, LDO3),
	AXP_MATCH(AXP20X, ldo4, LDO4),
	AXP_MATCH(AXP20X, ldo5, LDO5),
};

static struct of_regulator_match axp22x_matches[] = {
	AXP_MATCH(AXP22X, dcdc1, DCDC1),
	AXP_MATCH(AXP22X, dcdc2, DCDC2),
	AXP_MATCH(AXP22X, dcdc3, DCDC3),
	AXP_MATCH(AXP22X, dcdc4, DCDC4),
	AXP_MATCH(AXP22X, dcdc5, DCDC5),
	AXP_MATCH(AXP22X, dc5ldo, DC5LDO),
	AXP_MATCH(AXP22X, aldo1, ALDO1),
	AXP_MATCH(AXP22X, aldo2, ALDO2),
	AXP_MATCH(AXP22X, aldo3, ALDO3),
	AXP_MATCH(AXP22X, dldo1, DLDO1),
	AXP_MATCH(AXP22X, dldo2, DLDO2),
	AXP_MATCH(AXP22X, dldo3, DLDO3),
	AXP_MATCH(AXP22X, dldo4, DLDO4),
	AXP_MATCH(AXP22X, eldo1, ELDO1),
	AXP_MATCH(AXP22X, eldo2, ELDO2),
	AXP_MATCH(AXP22X, eldo3, ELDO3),
	AXP_MATCH(AXP22X, ldo_io0, LDO_IO0),
	AXP_MATCH(AXP22X, ldo_io1, LDO_IO1),
	AXP_MATCH(AXP22X, rtc_ldo, RTC_LDO),
};

static int axp20x_set_dcdc_freq(struct platform_device *pdev, u32 dcdcfreq)
{
	struct axp20x_dev *axp20x = dev_get_drvdata(pdev->dev.parent);

	if (dcdcfreq < 750) {
		dcdcfreq = 750;
		dev_warn(&pdev->dev, "DCDC frequency too low. Set to 750kHz\n");
	}

	if (dcdcfreq > 1875) {
		dcdcfreq = 1875;
		dev_warn(&pdev->dev, "DCDC frequency too high. Set to 1875kHz\n");
	}

	dcdcfreq = (dcdcfreq - 750) / 75;

	return regmap_update_bits(axp20x->regmap, AXP20X_DCDC_FREQ,
				  AXP20X_FREQ_DCDC_MASK, dcdcfreq);
}

static int axp20x_regulator_parse_dt(struct platform_device *pdev,
				     struct of_regulator_match *matches,
				     int nmatches)
{
	struct device_node *np, *regulators;
	int ret;
	u32 dcdcfreq;

	np = of_node_get(pdev->dev.parent->of_node);
	if (!np)
		return 0;

	regulators = of_get_child_by_name(np, "regulators");
	if (!regulators) {
		dev_warn(&pdev->dev, "regulators node not found\n");
	} else {
		ret = of_regulator_match(&pdev->dev, regulators, matches,
					 nmatches);
		if (ret < 0) {
			dev_err(&pdev->dev, "Error parsing regulator init data: %d\n", ret);
			return ret;
		}

		dcdcfreq = 1500;
		of_property_read_u32(regulators, "x-powers,dcdc-freq", &dcdcfreq);
		ret = axp20x_set_dcdc_freq(pdev, dcdcfreq);
		if (ret < 0) {
			dev_err(&pdev->dev, "Error setting dcdc frequency: %d\n", ret);
			return ret;
		}

		of_node_put(regulators);
	}

	return 0;
}

static int axp20x_set_dcdc_workmode(struct regulator_dev *rdev, int id, u32 workmode)
{
	unsigned int mask = AXP20X_WORKMODE_DCDC2_MASK;

	if ((id != AXP20X_DCDC2) && (id != AXP20X_DCDC3))
		return -EINVAL;

	if (id == AXP20X_DCDC3)
		mask = AXP20X_WORKMODE_DCDC3_MASK;

	workmode <<= ffs(mask) - 1;

	return regmap_update_bits(rdev->regmap, AXP20X_DCDC_MODE, mask, workmode);
}

static int axp20x_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;
	struct axp20x_dev *axp20x = dev_get_drvdata(pdev->dev.parent);
	struct regulator_config config = { };
	struct regulator_init_data *init_data;
	struct of_regulator_match *matches;
	int nmatches;
	const struct regulator_desc *regulators;
	int nregulators;
	int ret, i;
	u32 workmode;

	ret = mfd_register_supply_aliases(pdev);
	if (ret)
		return ret;

	if (axp20x->variant == AXP221_ID) {
		matches = axp22x_matches;
		nmatches = ARRAY_SIZE(axp22x_matches);
		regulators = axp22x_regulators;
		nregulators = AXP22X_REG_ID_MAX;
	} else {
		matches = axp20x_matches;
		nmatches = ARRAY_SIZE(axp20x_matches);
		regulators = axp20x_regulators;
		nregulators = AXP20X_REG_ID_MAX;
	}

	ret = axp20x_regulator_parse_dt(pdev, matches, nmatches);
	if (ret)
		return ret;

	for (i = 0; i < AXP20X_REG_ID_MAX; i++) {
		init_data = matches[i].init_data;

		config.dev = &pdev->dev;
		config.init_data = init_data;
		config.regmap = axp20x->regmap;
		config.of_node = matches[i].of_node;

		rdev = devm_regulator_register(&pdev->dev, &regulators[i],
					       &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev, "Failed to register %s\n",
				regulators[i].name);

			return PTR_ERR(rdev);
		}

		ret = of_property_read_u32(matches[i].of_node, "x-powers,dcdc-workmode",
					   &workmode);
		if (!ret) {
			if (axp20x_set_dcdc_workmode(rdev, i, workmode))
				dev_err(&pdev->dev, "Failed to set workmode on %s\n",
					regulators[i].name);
		}
	}

	return 0;
}

static struct platform_driver axp20x_regulator_driver = {
	.probe	= axp20x_regulator_probe,
	.driver	= {
		.name		= "axp20x-regulator",
		.owner		= THIS_MODULE,
	},
};

module_platform_driver(axp20x_regulator_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Carlo Caione <carlo@caione.org>");
MODULE_DESCRIPTION("Regulator Driver for AXP20X PMIC");
