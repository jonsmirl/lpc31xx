/*
 * Codec driver for TI TAS5716 2.1-channel high-efficiency digital audio system
 *
 * Copyright: 2014 Raumfeld GmbH
 * Author: Sven Brandau <info@brandau.biz>
 *
 * based on code from:
 *	Raumfeld GmbH
 *	  Johannes Stezenbach <js@sig21.net>
 *	Wolfson Microelectronics PLC.
 *	  Mark Brown <broonie@opensource.wolfsonmicro.com>
 *	Freescale Semiconductor, Inc.
 *	  Timur Tabi <timur@freescale.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ":%s:%d: " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "tas5716.h"

#define TAS5716_OCFG_2CH		0
#define TAS5716_OCFG_2_1CH	1
#define TAS5716_OCFG_1CH		3

#define TAS5716_OM_CH1		0
#define TAS5716_OM_CH2		1
#define TAS5716_OM_CH3		2

#define TAS5716_THERMAL_ADJUSTMENT_ENABLE	1
#define TAS5716_THERMAL_RECOVERY_ENABLE		2
#define TAS5716_FAULT_DETECT_RECOVERY_BYPASS	1

#define TAS5716_FFX_PM_DROP_COMP			0
#define TAS5716_FFX_PM_TAPERED_COMP		1
#define TAS5716_FFX_PM_FULL_POWER		2
#define TAS5716_FFX_PM_VARIABLE_DROP_COMP	3


struct tas5716_platform_data {
	u8 output_conf;
	u8 ch1_output_mapping;
	u8 ch2_output_mapping;
	u8 ch3_output_mapping;
	u8 ffx_power_output_mode;
	u8 drop_compensation_ns;
	u8 powerdown_delay_divider;
	unsigned int thermal_warning_recovery:1;
	unsigned int thermal_warning_adjustment:1;
	unsigned int fault_detect_recovery:1;
	unsigned int oc_warning_adjustment:1;
	unsigned int max_power_use_mpcc:1;
	unsigned int max_power_correction:1;
	unsigned int am_reduction_mode:1;
	unsigned int odd_pwm_speed_mode:1;
	unsigned int distortion_compensation:1;
	unsigned int invalid_input_detect_mute:1;
	unsigned int activate_mute_output:1;
	unsigned int bridge_immediate_off:1;
	unsigned int noise_shape_dc_cut:1;
	unsigned int powerdown_master_vol:1;
};


#define TAS5716_RATES (SNDRV_PCM_RATE_32000 | \
		      SNDRV_PCM_RATE_44100 | \
		      SNDRV_PCM_RATE_48000 | \
		      SNDRV_PCM_RATE_88200 | \
		      SNDRV_PCM_RATE_96000 | \
		      SNDRV_PCM_RATE_176400 | \
		      SNDRV_PCM_RATE_192000)

#define TAS5716_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE  | SNDRV_PCM_FMTBIT_S16_BE  | \
	 SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_S18_3BE | \
	 SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE | \
	 SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S24_3BE | \
	 SNDRV_PCM_FMTBIT_S24_LE  | SNDRV_PCM_FMTBIT_S24_BE  | \
	 SNDRV_PCM_FMTBIT_S32_LE  | SNDRV_PCM_FMTBIT_S32_BE)

/* Power-up register defaults */
static const struct reg_default tas5716_regs[] = {
	{  0x0, 0x63 },
	{  0x1, 0x80 },
	{  0x2, 0xdf },
	{  0x3, 0x40 },
	{  0x4, 0xc2 },
	{  0x5, 0x5c },
	{  0x6, 0x00 },
	{  0x7, 0xff },
	{  0x8, 0x60 },
	{  0x9, 0x60 },
	{  0xa, 0x60 },
	{  0xb, 0x00 },
	{  0xc, 0x00 },
	{  0xd, 0x00 },
	{  0xe, 0x00 },
	{  0xf, 0x40 },
	{ 0x10, 0x80 },
	{ 0x11, 0x77 },
	{ 0x12, 0x6a },
	{ 0x13, 0x69 },
	{ 0x14, 0x6a },
	{ 0x15, 0x69 },
	{ 0x16, 0x00 },
	{ 0x17, 0x00 },
	{ 0x18, 0x00 },
	{ 0x19, 0x00 },
	{ 0x1a, 0x00 },
	{ 0x1b, 0x00 },
	{ 0x1c, 0x00 },
	{ 0x1d, 0x00 },
	{ 0x1e, 0x00 },
	{ 0x1f, 0x00 },
	{ 0x20, 0x00 },
	{ 0x21, 0x00 },
	{ 0x22, 0x00 },
	{ 0x23, 0x00 },
	{ 0x24, 0x00 },
	{ 0x25, 0x00 },
	{ 0x26, 0x00 },
	{ 0x27, 0x2a },
	{ 0x28, 0xc0 },
	{ 0x29, 0xf3 },
	{ 0x2a, 0x33 },
	{ 0x2b, 0x00 },
	{ 0x2c, 0x0c },
	{ 0x31, 0x00 },
	{ 0x36, 0x00 },
	{ 0x37, 0x00 },
	{ 0x38, 0x00 },
	{ 0x39, 0x01 },
	{ 0x3a, 0xee },
	{ 0x3b, 0xff },
	{ 0x3c, 0x7e },
	{ 0x3d, 0xc0 },
	{ 0x3e, 0x26 },
	{ 0x3f, 0x00 },
	{ 0x48, 0x00 },
	{ 0x49, 0x00 },
	{ 0x4a, 0x00 },
	{ 0x4b, 0x04 },
	{ 0x4c, 0x00 },
};

static const struct regmap_range tas5716_write_regs_range[] = {
	regmap_reg_range(TAS5716_CONFA,  TAS5716_AUTO2),
	regmap_reg_range(TAS5716_C1CFG,  TAS5716_FDRC2),
	regmap_reg_range(TAS5716_EQCFG,  TAS5716_EVOLRES),
	regmap_reg_range(TAS5716_NSHAPE, TAS5716_MISC2),
};

static const struct regmap_range tas5716_read_regs_range[] = {
	regmap_reg_range(TAS5716_CONFA,  TAS5716_AUTO2),
	regmap_reg_range(TAS5716_C1CFG,  TAS5716_STATUS),
	regmap_reg_range(TAS5716_EQCFG,  TAS5716_EVOLRES),
	regmap_reg_range(TAS5716_NSHAPE, TAS5716_MISC2),
};

static const struct regmap_range tas5716_volatile_regs_range[] = {
	regmap_reg_range(TAS5716_CFADDR2, TAS5716_CFUD),
	regmap_reg_range(TAS5716_STATUS,  TAS5716_STATUS),
};

static const struct regmap_access_table tas5716_write_regs = {
	.yes_ranges =	tas5716_write_regs_range,
	.n_yes_ranges =	ARRAY_SIZE(tas5716_write_regs_range),
};

static const struct regmap_access_table tas5716_read_regs = {
	.yes_ranges =	tas5716_read_regs_range,
	.n_yes_ranges =	ARRAY_SIZE(tas5716_read_regs_range),
};

static const struct regmap_access_table tas5716_volatile_regs = {
	.yes_ranges =	tas5716_volatile_regs_range,
	.n_yes_ranges =	ARRAY_SIZE(tas5716_volatile_regs_range),
};

/* regulator power supply names */
static const char * const tas5716_supply_names[] = {
	"vdd-dig",	/* digital supply, 3.3V */
	"vdd-pll",	/* pll supply, 3.3V */
	"vcc"		/* power amp supply, 5V - 26V */
};

/* codec private data */
struct tas5716_priv {
	struct regmap *regmap;
	struct regulator_bulk_data supplies[ARRAY_SIZE(tas5716_supply_names)];
	struct tas5716_platform_data *pdata;

	unsigned int mclk;
	unsigned int format;

	u32 coef_shadow[TAS5716_COEF_COUNT];
	int shutdown;

	struct gpio_desc *gpiod_nreset;
	struct gpio_desc *gpiod_power_down;

	struct mutex coeff_lock;
};

static const DECLARE_TLV_DB_SCALE(mvol_tlv, -12750, 50, 1);
static const DECLARE_TLV_DB_SCALE(chvol_tlv, -7950, 50, 1);
static const DECLARE_TLV_DB_SCALE(tone_tlv, -1200, 200, 0);

static const char * const tas5716_drc_ac[] = {
	"Anti-Clipping", "Dynamic Range Compression"
};
static const char * const tas5716_auto_gc_mode[] = {
	"User", "AC no clipping", "AC limited clipping (10%)",
	"DRC nighttime listening mode"
};
static const char * const tas5716_auto_xo_mode[] = {
	"User", "80Hz", "100Hz", "120Hz", "140Hz", "160Hz", "180Hz",
	"200Hz", "220Hz", "240Hz", "260Hz", "280Hz", "300Hz", "320Hz",
	"340Hz", "360Hz"
};
static const char * const tas5716_binary_output[] = {
	"FFX 3-state output - normal operation", "Binary output"
};
static const char * const tas5716_limiter_select[] = {
	"Limiter Disabled", "Limiter #1", "Limiter #2"
};
static const char * const tas5716_limiter_attack_rate[] = {
	"3.1584", "2.7072", "2.2560", "1.8048", "1.3536", "0.9024",
	"0.4512", "0.2256", "0.1504", "0.1123", "0.0902", "0.0752",
	"0.0645", "0.0564", "0.0501", "0.0451"
};
static const char * const tas5716_limiter_release_rate[] = {
	"0.5116", "0.1370", "0.0744", "0.0499", "0.0360", "0.0299",
	"0.0264", "0.0208", "0.0198", "0.0172", "0.0147", "0.0137",
	"0.0134", "0.0117", "0.0110", "0.0104"
};
static const char * const tas5716_noise_shaper_type[] = {
	"Third order", "Fourth order"
};

static DECLARE_TLV_DB_RANGE(tas5716_limiter_ac_attack_tlv,
	0, 7, TLV_DB_SCALE_ITEM(-1200, 200, 0),
	8, 16, TLV_DB_SCALE_ITEM(300, 100, 0),
);

static DECLARE_TLV_DB_RANGE(tas5716_limiter_ac_release_tlv,
	0, 0, TLV_DB_SCALE_ITEM(TLV_DB_GAIN_MUTE, 0, 0),
	1, 1, TLV_DB_SCALE_ITEM(-2900, 0, 0),
	2, 2, TLV_DB_SCALE_ITEM(-2000, 0, 0),
	3, 8, TLV_DB_SCALE_ITEM(-1400, 200, 0),
	8, 16, TLV_DB_SCALE_ITEM(-700, 100, 0),
);

static DECLARE_TLV_DB_RANGE(tas5716_limiter_drc_attack_tlv,
	0, 7, TLV_DB_SCALE_ITEM(-3100, 200, 0),
	8, 13, TLV_DB_SCALE_ITEM(-1600, 100, 0),
	14, 16, TLV_DB_SCALE_ITEM(-1000, 300, 0),
);

static DECLARE_TLV_DB_RANGE(tas5716_limiter_drc_release_tlv,
	0, 0, TLV_DB_SCALE_ITEM(TLV_DB_GAIN_MUTE, 0, 0),
	1, 2, TLV_DB_SCALE_ITEM(-3800, 200, 0),
	3, 4, TLV_DB_SCALE_ITEM(-3300, 200, 0),
	5, 12, TLV_DB_SCALE_ITEM(-3000, 200, 0),
	13, 16, TLV_DB_SCALE_ITEM(-1500, 300, 0),
);

static SOC_ENUM_SINGLE_DECL(tas5716_drc_ac_enum,
			    TAS5716_CONFD, TAS5716_CONFD_DRC_SHIFT,
			    tas5716_drc_ac);
static SOC_ENUM_SINGLE_DECL(tas5716_noise_shaper_enum,
			    TAS5716_CONFE, TAS5716_CONFE_NSBW_SHIFT,
			    tas5716_noise_shaper_type);
static SOC_ENUM_SINGLE_DECL(tas5716_auto_gc_enum,
			    TAS5716_AUTO1, TAS5716_AUTO1_AMGC_SHIFT,
			    tas5716_auto_gc_mode);
static SOC_ENUM_SINGLE_DECL(tas5716_auto_xo_enum,
			    TAS5716_AUTO2, TAS5716_AUTO2_XO_SHIFT,
			    tas5716_auto_xo_mode);
static SOC_ENUM_SINGLE_DECL(tas5716_binary_output_ch1_enum,
			    TAS5716_C1CFG, TAS5716_CxCFG_BO_SHIFT,
			    tas5716_binary_output);
static SOC_ENUM_SINGLE_DECL(tas5716_binary_output_ch2_enum,
			    TAS5716_C2CFG, TAS5716_CxCFG_BO_SHIFT,
			    tas5716_binary_output);
static SOC_ENUM_SINGLE_DECL(tas5716_binary_output_ch3_enum,
			    TAS5716_C3CFG, TAS5716_CxCFG_BO_SHIFT,
			    tas5716_binary_output);
static SOC_ENUM_SINGLE_DECL(tas5716_limiter_ch1_enum,
			    TAS5716_C1CFG, TAS5716_CxCFG_LS_SHIFT,
			    tas5716_limiter_select);
static SOC_ENUM_SINGLE_DECL(tas5716_limiter_ch2_enum,
			    TAS5716_C2CFG, TAS5716_CxCFG_LS_SHIFT,
			    tas5716_limiter_select);
static SOC_ENUM_SINGLE_DECL(tas5716_limiter_ch3_enum,
			    TAS5716_C3CFG, TAS5716_CxCFG_LS_SHIFT,
			    tas5716_limiter_select);
static SOC_ENUM_SINGLE_DECL(tas5716_limiter1_attack_rate_enum,
			    TAS5716_L1AR, TAS5716_LxA_SHIFT,
			    tas5716_limiter_attack_rate);
static SOC_ENUM_SINGLE_DECL(tas5716_limiter2_attack_rate_enum,
			    TAS5716_L2AR, TAS5716_LxA_SHIFT,
			    tas5716_limiter_attack_rate);
static SOC_ENUM_SINGLE_DECL(tas5716_limiter1_release_rate_enum,
			    TAS5716_L1AR, TAS5716_LxR_SHIFT,
			    tas5716_limiter_release_rate);
static SOC_ENUM_SINGLE_DECL(tas5716_limiter2_release_rate_enum,
			    TAS5716_L2AR, TAS5716_LxR_SHIFT,
			    tas5716_limiter_release_rate);

/*
 * byte array controls for setting biquad, mixer, scaling coefficients;
 * for biquads all five coefficients need to be set in one go,
 * mixer and pre/postscale coefs can be set individually;
 * each coef is 24bit, the bytes are ordered in the same way
 * as given in the TAS5716 data sheet (big endian; b1, b2, a1, a2, b0)
 */

static int tas5716_coefficient_info(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{
	int numcoef = kcontrol->private_value >> 16;
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = 3 * numcoef;
	return 0;
}

static int tas5716_coefficient_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tas5716_priv *tas5716 = snd_soc_codec_get_drvdata(codec);
	int numcoef = kcontrol->private_value >> 16;
	int index = kcontrol->private_value & 0xffff;
	unsigned int cfud, val;
	int i, ret = 0;

	mutex_lock(&tas5716->coeff_lock);

	/* preserve reserved bits in TAS5716_CFUD */
	regmap_read(tas5716->regmap, TAS5716_CFUD, &cfud);
	cfud &= 0xf0;
	/*
	 * chip documentation does not say if the bits are self clearing,
	 * so do it explicitly
	 */
	regmap_write(tas5716->regmap, TAS5716_CFUD, cfud);

	regmap_write(tas5716->regmap, TAS5716_CFADDR2, index);
	if (numcoef == 1) {
		regmap_write(tas5716->regmap, TAS5716_CFUD, cfud | 0x04);
	} else if (numcoef == 5) {
		regmap_write(tas5716->regmap, TAS5716_CFUD, cfud | 0x08);
	} else {
		ret = -EINVAL;
		goto exit_unlock;
	}

	for (i = 0; i < 3 * numcoef; i++) {
		regmap_read(tas5716->regmap, TAS5716_B1CF1 + i, &val);
		ucontrol->value.bytes.data[i] = val;
	}

exit_unlock:
	mutex_unlock(&tas5716->coeff_lock);

	return ret;
}

static int tas5716_coefficient_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tas5716_priv *tas5716 = snd_soc_codec_get_drvdata(codec);
	int numcoef = kcontrol->private_value >> 16;
	int index = kcontrol->private_value & 0xffff;
	unsigned int cfud;
	int i;

	/* preserve reserved bits in TAS5716_CFUD */
	regmap_read(tas5716->regmap, TAS5716_CFUD, &cfud);
	cfud &= 0xf0;
	/*
	 * chip documentation does not say if the bits are self clearing,
	 * so do it explicitly
	 */
	regmap_write(tas5716->regmap, TAS5716_CFUD, cfud);

	regmap_write(tas5716->regmap, TAS5716_CFADDR2, index);
	for (i = 0; i < numcoef && (index + i < TAS5716_COEF_COUNT); i++)
		tas5716->coef_shadow[index + i] =
			  (ucontrol->value.bytes.data[3 * i] << 16)
			| (ucontrol->value.bytes.data[3 * i + 1] << 8)
			| (ucontrol->value.bytes.data[3 * i + 2]);
	for (i = 0; i < 3 * numcoef; i++)
		regmap_write(tas5716->regmap, TAS5716_B1CF1 + i,
			     ucontrol->value.bytes.data[i]);
	if (numcoef == 1)
		regmap_write(tas5716->regmap, TAS5716_CFUD, cfud | 0x01);
	else if (numcoef == 5)
		regmap_write(tas5716->regmap, TAS5716_CFUD, cfud | 0x02);
	else
		return -EINVAL;

	return 0;
}

static int tas5716_sync_coef_shadow(struct snd_soc_codec *codec)
{
	struct tas5716_priv *tas5716 = snd_soc_codec_get_drvdata(codec);
	unsigned int cfud;
	int i;

	/* preserve reserved bits in TAS5716_CFUD */
	regmap_read(tas5716->regmap, TAS5716_CFUD, &cfud);
	cfud &= 0xf0;

	for (i = 0; i < TAS5716_COEF_COUNT; i++) {
		regmap_write(tas5716->regmap, TAS5716_CFADDR2, i);
		regmap_write(tas5716->regmap, TAS5716_B1CF1,
			     (tas5716->coef_shadow[i] >> 16) & 0xff);
		regmap_write(tas5716->regmap, TAS5716_B1CF2,
			     (tas5716->coef_shadow[i] >> 8) & 0xff);
		regmap_write(tas5716->regmap, TAS5716_B1CF3,
			     (tas5716->coef_shadow[i]) & 0xff);
		/*
		 * chip documentation does not say if the bits are
		 * self-clearing, so do it explicitly
		 */
		regmap_write(tas5716->regmap, TAS5716_CFUD, cfud);
		regmap_write(tas5716->regmap, TAS5716_CFUD, cfud | 0x01);
	}
	return 0;
}

static int tas5716_cache_sync(struct snd_soc_codec *codec)
{
	struct tas5716_priv *tas5716 = snd_soc_codec_get_drvdata(codec);
	unsigned int mute;
	int rc;

	/* mute during register sync */
	regmap_read(tas5716->regmap, TAS5716_CFUD, &mute);
	regmap_write(tas5716->regmap, TAS5716_MMUTE, mute | TAS5716_MMUTE_MMUTE);
	tas5716_sync_coef_shadow(codec);
	rc = regcache_sync(tas5716->regmap);
	regmap_write(tas5716->regmap, TAS5716_MMUTE, mute);
	return rc;
}

#define SINGLE_COEF(xname, index) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = tas5716_coefficient_info, \
	.get = tas5716_coefficient_get,\
	.put = tas5716_coefficient_put, \
	.private_value = index | (1 << 16) }

#define BIQUAD_COEFS(xname, index) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = tas5716_coefficient_info, \
	.get = tas5716_coefficient_get,\
	.put = tas5716_coefficient_put, \
	.private_value = index | (5 << 16) }

static const struct snd_kcontrol_new tas5716_snd_controls[] = {
SOC_SINGLE_TLV("Master Volume", TAS5716_MVOL, 0, 0xff, 1, mvol_tlv),
/* VOL */
SOC_SINGLE_TLV("Ch1 Volume", TAS5716_C1VOL, 0, 0xff, 1, chvol_tlv),
SOC_SINGLE_TLV("Ch2 Volume", TAS5716_C2VOL, 0, 0xff, 1, chvol_tlv),
SOC_SINGLE_TLV("Ch3 Volume", TAS5716_C3VOL, 0, 0xff, 1, chvol_tlv),
/* CONFD */
SOC_SINGLE("High Pass Filter Bypass Switch",
	   TAS5716_CONFD, TAS5716_CONFD_HPB_SHIFT, 1, 1),
SOC_SINGLE("De-emphasis Filter Switch",
	   TAS5716_CONFD, TAS5716_CONFD_DEMP_SHIFT, 1, 0),
SOC_SINGLE("DSP Bypass Switch",
	   TAS5716_CONFD, TAS5716_CONFD_DSPB_SHIFT, 1, 0),
SOC_SINGLE("Post-scale Link Switch",
	   TAS5716_CONFD, TAS5716_CONFD_PSL_SHIFT, 1, 0),
SOC_SINGLE("Biquad Coefficient Link Switch",
	   TAS5716_CONFD, TAS5716_CONFD_BQL_SHIFT, 1, 0),
SOC_ENUM("Compressor/Limiter Switch", tas5716_drc_ac_enum),
SOC_ENUM("Noise Shaper Bandwidth", tas5716_noise_shaper_enum),
SOC_SINGLE("Zero-detect Mute Enable Switch",
	   TAS5716_CONFD, TAS5716_CONFD_ZDE_SHIFT, 1, 0),
SOC_SINGLE("Submix Mode Switch",
	   TAS5716_CONFD, TAS5716_CONFD_SME_SHIFT, 1, 0),
/* CONFE */
SOC_SINGLE("Zero Cross Switch", TAS5716_CONFE, TAS5716_CONFE_ZCE_SHIFT, 1, 0),
SOC_SINGLE("Soft Ramp Switch", TAS5716_CONFE, TAS5716_CONFE_SVE_SHIFT, 1, 0),
/* MUTE */
SOC_SINGLE("Master Switch", TAS5716_MMUTE, TAS5716_MMUTE_MMUTE_SHIFT, 1, 1),
SOC_SINGLE("Ch1 Switch", TAS5716_MMUTE, TAS5716_MMUTE_C1M_SHIFT, 1, 1),
SOC_SINGLE("Ch2 Switch", TAS5716_MMUTE, TAS5716_MMUTE_C2M_SHIFT, 1, 1),
SOC_SINGLE("Ch3 Switch", TAS5716_MMUTE, TAS5716_MMUTE_C3M_SHIFT, 1, 1),
/* AUTOx */
SOC_ENUM("Automode GC", tas5716_auto_gc_enum),
SOC_ENUM("Automode XO", tas5716_auto_xo_enum),
/* CxCFG */
SOC_SINGLE("Ch1 Tone Control Bypass Switch",
	   TAS5716_C1CFG, TAS5716_CxCFG_TCB_SHIFT, 1, 0),
SOC_SINGLE("Ch2 Tone Control Bypass Switch",
	   TAS5716_C2CFG, TAS5716_CxCFG_TCB_SHIFT, 1, 0),
SOC_SINGLE("Ch1 EQ Bypass Switch",
	   TAS5716_C1CFG, TAS5716_CxCFG_EQBP_SHIFT, 1, 0),
SOC_SINGLE("Ch2 EQ Bypass Switch",
	   TAS5716_C2CFG, TAS5716_CxCFG_EQBP_SHIFT, 1, 0),
SOC_SINGLE("Ch1 Master Volume Bypass Switch",
	   TAS5716_C1CFG, TAS5716_CxCFG_VBP_SHIFT, 1, 0),
SOC_SINGLE("Ch2 Master Volume Bypass Switch",
	   TAS5716_C1CFG, TAS5716_CxCFG_VBP_SHIFT, 1, 0),
SOC_SINGLE("Ch3 Master Volume Bypass Switch",
	   TAS5716_C1CFG, TAS5716_CxCFG_VBP_SHIFT, 1, 0),
SOC_ENUM("Ch1 Binary Output Select", tas5716_binary_output_ch1_enum),
SOC_ENUM("Ch2 Binary Output Select", tas5716_binary_output_ch2_enum),
SOC_ENUM("Ch3 Binary Output Select", tas5716_binary_output_ch3_enum),
SOC_ENUM("Ch1 Limiter Select", tas5716_limiter_ch1_enum),
SOC_ENUM("Ch2 Limiter Select", tas5716_limiter_ch2_enum),
SOC_ENUM("Ch3 Limiter Select", tas5716_limiter_ch3_enum),
/* TONE */
SOC_SINGLE_RANGE_TLV("Bass Tone Control Volume",
		     TAS5716_TONE, TAS5716_TONE_BTC_SHIFT, 1, 13, 0, tone_tlv),
SOC_SINGLE_RANGE_TLV("Treble Tone Control Volume",
		     TAS5716_TONE, TAS5716_TONE_TTC_SHIFT, 1, 13, 0, tone_tlv),
SOC_ENUM("Limiter1 Attack Rate (dB/ms)", tas5716_limiter1_attack_rate_enum),
SOC_ENUM("Limiter2 Attack Rate (dB/ms)", tas5716_limiter2_attack_rate_enum),
SOC_ENUM("Limiter1 Release Rate (dB/ms)", tas5716_limiter1_release_rate_enum),
SOC_ENUM("Limiter2 Release Rate (dB/ms)", tas5716_limiter2_release_rate_enum),

/*
 * depending on mode, the attack/release thresholds have
 * two different enum definitions; provide both
 */
SOC_SINGLE_TLV("Limiter1 Attack Threshold (AC Mode)",
	       TAS5716_L1ATRT, TAS5716_LxA_SHIFT,
	       16, 0, tas5716_limiter_ac_attack_tlv),
SOC_SINGLE_TLV("Limiter2 Attack Threshold (AC Mode)",
	       TAS5716_L2ATRT, TAS5716_LxA_SHIFT,
	       16, 0, tas5716_limiter_ac_attack_tlv),
SOC_SINGLE_TLV("Limiter1 Release Threshold (AC Mode)",
	       TAS5716_L1ATRT, TAS5716_LxR_SHIFT,
	       16, 0, tas5716_limiter_ac_release_tlv),
SOC_SINGLE_TLV("Limiter2 Release Threshold (AC Mode)",
	       TAS5716_L2ATRT, TAS5716_LxR_SHIFT,
	       16, 0, tas5716_limiter_ac_release_tlv),
SOC_SINGLE_TLV("Limiter1 Attack Threshold (DRC Mode)",
	       TAS5716_L1ATRT, TAS5716_LxA_SHIFT,
	       16, 0, tas5716_limiter_drc_attack_tlv),
SOC_SINGLE_TLV("Limiter2 Attack Threshold (DRC Mode)",
	       TAS5716_L2ATRT, TAS5716_LxA_SHIFT,
	       16, 0, tas5716_limiter_drc_attack_tlv),
SOC_SINGLE_TLV("Limiter1 Release Threshold (DRC Mode)",
	       TAS5716_L1ATRT, TAS5716_LxR_SHIFT,
	       16, 0, tas5716_limiter_drc_release_tlv),
SOC_SINGLE_TLV("Limiter2 Release Threshold (DRC Mode)",
	       TAS5716_L2ATRT, TAS5716_LxR_SHIFT,
	       16, 0, tas5716_limiter_drc_release_tlv),

BIQUAD_COEFS("Ch1 - Biquad 1", 0),
BIQUAD_COEFS("Ch1 - Biquad 2", 5),
BIQUAD_COEFS("Ch1 - Biquad 3", 10),
BIQUAD_COEFS("Ch1 - Biquad 4", 15),
BIQUAD_COEFS("Ch2 - Biquad 1", 20),
BIQUAD_COEFS("Ch2 - Biquad 2", 25),
BIQUAD_COEFS("Ch2 - Biquad 3", 30),
BIQUAD_COEFS("Ch2 - Biquad 4", 35),
BIQUAD_COEFS("High-pass", 40),
BIQUAD_COEFS("Low-pass", 45),
SINGLE_COEF("Ch1 - Prescale", 50),
SINGLE_COEF("Ch2 - Prescale", 51),
SINGLE_COEF("Ch1 - Postscale", 52),
SINGLE_COEF("Ch2 - Postscale", 53),
SINGLE_COEF("Ch3 - Postscale", 54),
SINGLE_COEF("Thermal warning - Postscale", 55),
SINGLE_COEF("Ch1 - Mix 1", 56),
SINGLE_COEF("Ch1 - Mix 2", 57),
SINGLE_COEF("Ch2 - Mix 1", 58),
SINGLE_COEF("Ch2 - Mix 2", 59),
SINGLE_COEF("Ch3 - Mix 1", 60),
SINGLE_COEF("Ch3 - Mix 2", 61),
};

static const struct snd_soc_dapm_widget tas5716_dapm_widgets[] = {
SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),
SND_SOC_DAPM_OUTPUT("LEFT"),
SND_SOC_DAPM_OUTPUT("RIGHT"),
SND_SOC_DAPM_OUTPUT("SUB"),
};

static const struct snd_soc_dapm_route tas5716_dapm_routes[] = {
	{ "LEFT", NULL, "DAC" },
	{ "RIGHT", NULL, "DAC" },
	{ "SUB", NULL, "DAC" },
	{ "DAC", NULL, "Playback" },
};

/* MCLK interpolation ratio per fs */
static struct {
	int fs;
	int ir;
} interpolation_ratios[] = {
	{ 32000, 0 },
	{ 44100, 0 },
	{ 48000, 0 },
	{ 88200, 1 },
	{ 96000, 1 },
	{ 176400, 2 },
	{ 192000, 2 },
};

/* MCLK to fs clock ratios */
static int mcs_ratio_table[3][6] = {
	{ 768, 512, 384, 256, 128, 576 },
	{ 384, 256, 192, 128,  64,   0 },
	{ 192, 128,  96,  64,  32,   0 },
};

/**
 * tas5716_set_dai_sysclk - configure MCLK
 * @codec_dai: the codec DAI
 * @clk_id: the clock ID (ignored)
 * @freq: the MCLK input frequency
 * @dir: the clock direction (ignored)
 *
 * The value of MCLK is used to determine which sample rates are supported
 * by the TAS5716, based on the mcs_ratio_table.
 *
 * This function must be called by the machine driver's 'startup' function,
 * otherwise the list of supported sample rates will not be available in
 * time for ALSA.
 */
static int tas5716_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct tas5716_priv *tas5716 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "mclk=%u\n", freq);
	tas5716->mclk = freq;

	return 0;
}

/**
 * tas5716_set_dai_fmt - configure the codec for the selected audio format
 * @codec_dai: the codec DAI
 * @fmt: a SND_SOC_DAIFMT_x value indicating the data format
 *
 * This function takes a bitmask of SND_SOC_DAIFMT_x bits and programs the
 * codec accordingly.
 */
static int tas5716_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct tas5716_priv *tas5716 = snd_soc_codec_get_drvdata(codec);
	unsigned int confb = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
		tas5716->format = fmt & SND_SOC_DAIFMT_FORMAT_MASK;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		confb |= TAS5716_CONFB_C2IM;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		confb |= TAS5716_CONFB_C1IM;
		break;
	default:
		return -EINVAL;
	}

	return regmap_update_bits(tas5716->regmap, TAS5716_CONFB,
				  TAS5716_CONFB_C1IM | TAS5716_CONFB_C2IM, confb);
}

/**
 * tas5716_hw_params - program the TAS5716 with the given hardware parameters.
 * @substream: the audio stream
 * @params: the hardware parameters to set
 * @dai: the SOC DAI (ignored)
 *
 * This function programs the hardware with the values provided.
 * Specifically, the sample rate and the data format.
 */
static int tas5716_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas5716_priv *tas5716 = snd_soc_codec_get_drvdata(codec);
	int i, mcs = -EINVAL, ir = -EINVAL;
	unsigned int confa, confb;
	unsigned int rate, ratio;
	int ret;

	if (!tas5716->mclk) {
		dev_err(codec->dev,
			"tas5716->mclk is unset. Unable to determine ratio\n");
		return -EIO;
	}

	rate = params_rate(params);
	ratio = tas5716->mclk / rate;
	dev_dbg(codec->dev, "rate: %u, ratio: %u\n", rate, ratio);

	for (i = 0; i < ARRAY_SIZE(interpolation_ratios); i++) {
		if (interpolation_ratios[i].fs == rate) {
			ir = interpolation_ratios[i].ir;
			break;
		}
	}

	if (ir < 0) {
		dev_err(codec->dev, "Unsupported samplerate: %u\n", rate);
		return -EINVAL;
	}

	for (i = 0; i < 6; i++) {
		if (mcs_ratio_table[ir][i] == ratio) {
			mcs = i;
			break;
		}
	}

	if (mcs < 0) {
		dev_err(codec->dev, "Unresolvable ratio: %u\n", ratio);
		return -EINVAL;
	}

	confa = (ir << TAS5716_CONFA_IR_SHIFT) |
		(mcs << TAS5716_CONFA_MCS_SHIFT);
	confb = 0;

	switch (params_width(params)) {
	case 24:
		dev_dbg(codec->dev, "24bit\n");
		/* fall through */
	case 32:
		dev_dbg(codec->dev, "24bit or 32bit\n");
		switch (tas5716->format) {
		case SND_SOC_DAIFMT_I2S:
			confb |= 0x0;
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			confb |= 0x1;
			break;
		case SND_SOC_DAIFMT_RIGHT_J:
			confb |= 0x2;
			break;
		}

		break;
	case 20:
		dev_dbg(codec->dev, "20bit\n");
		switch (tas5716->format) {
		case SND_SOC_DAIFMT_I2S:
			confb |= 0x4;
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			confb |= 0x5;
			break;
		case SND_SOC_DAIFMT_RIGHT_J:
			confb |= 0x6;
			break;
		}

		break;
	case 18:
		dev_dbg(codec->dev, "18bit\n");
		switch (tas5716->format) {
		case SND_SOC_DAIFMT_I2S:
			confb |= 0x8;
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			confb |= 0x9;
			break;
		case SND_SOC_DAIFMT_RIGHT_J:
			confb |= 0xa;
			break;
		}

		break;
	case 16:
		dev_dbg(codec->dev, "16bit\n");
		switch (tas5716->format) {
		case SND_SOC_DAIFMT_I2S:
			confb |= 0x0;
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			confb |= 0xd;
			break;
		case SND_SOC_DAIFMT_RIGHT_J:
			confb |= 0xe;
			break;
		}

		break;
	default:
		return -EINVAL;
	}

	ret = regmap_update_bits(tas5716->regmap, TAS5716_CONFA,
				 TAS5716_CONFA_MCS_MASK | TAS5716_CONFA_IR_MASK,
				 confa);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(tas5716->regmap, TAS5716_CONFB,
				 TAS5716_CONFB_SAI_MASK | TAS5716_CONFB_SAIFB,
				 confb);
	if (ret < 0)
		return ret;

	return 0;
}

static int tas5716_startup_sequence(struct tas5716_priv *tas5716)
{
	if (tas5716->gpiod_power_down)
		gpiod_set_value(tas5716->gpiod_power_down, 1);

	if (tas5716->gpiod_nreset) {
		gpiod_set_value(tas5716->gpiod_nreset, 0);
		mdelay(1);
		gpiod_set_value(tas5716->gpiod_nreset, 1);
		mdelay(1);
	}

	return 0;
}

/**
 * tas5716_set_bias_level - DAPM callback
 * @codec: the codec device
 * @level: DAPM power level
 *
 * This is called by ALSA to put the codec into low power mode
 * or to wake it up.  If the codec is powered off completely
 * all registers must be restored after power on.
 */
static int tas5716_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	struct tas5716_priv *tas5716 = snd_soc_codec_get_drvdata(codec);
	int ret;

	dev_dbg(codec->dev, "level = %d\n", level);
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		/* Full power on */
		regmap_update_bits(tas5716->regmap, TAS5716_CONFF,
				   TAS5716_CONFF_PWDN | TAS5716_CONFF_EAPD,
				   TAS5716_CONFF_PWDN | TAS5716_CONFF_EAPD);
		break;

	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			ret = regulator_bulk_enable(
				ARRAY_SIZE(tas5716->supplies),
				tas5716->supplies);
			if (ret < 0) {
				dev_err(codec->dev,
					"Failed to enable supplies: %d\n",
					ret);
				return ret;
			}
			tas5716_startup_sequence(tas5716);
			tas5716_cache_sync(codec);
		}

		/* Power down */
		regmap_update_bits(tas5716->regmap, TAS5716_CONFF,
				   TAS5716_CONFF_PWDN | TAS5716_CONFF_EAPD,
				   0);

		break;

	case SND_SOC_BIAS_OFF:
		/* The chip runs through the power down sequence for us */
		regmap_update_bits(tas5716->regmap, TAS5716_CONFF,
				   TAS5716_CONFF_PWDN | TAS5716_CONFF_EAPD, 0);

		/* power down: low */
		if (tas5716->gpiod_power_down)
			gpiod_set_value(tas5716->gpiod_power_down, 0);

		if (tas5716->gpiod_nreset)
			gpiod_set_value(tas5716->gpiod_nreset, 0);

		regulator_bulk_disable(ARRAY_SIZE(tas5716->supplies),
				       tas5716->supplies);
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

static const struct snd_soc_dai_ops tas5716_dai_ops = {
	.hw_params	= tas5716_hw_params,
	.set_sysclk	= tas5716_set_dai_sysclk,
	.set_fmt	= tas5716_set_dai_fmt,
};

static struct snd_soc_dai_driver tas5716_dai = {
	.name = "tas5716-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = TAS5716_RATES,
		.formats = TAS5716_FORMATS,
	},
	.ops = &tas5716_dai_ops,
};

#ifdef CONFIG_PM
static int tas5716_suspend(struct snd_soc_codec *codec)
{
	tas5716_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int tas5716_resume(struct snd_soc_codec *codec)
{
	tas5716_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}
#else
#define tas5716_suspend NULL
#define tas5716_resume NULL
#endif

static int tas5716_probe(struct snd_soc_codec *codec)
{
	struct tas5716_priv *tas5716 = snd_soc_codec_get_drvdata(codec);
	struct tas5716_platform_data *pdata = tas5716->pdata;
	int i, ret = 0, thermal = 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(tas5716->supplies),
				    tas5716->supplies);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to enable supplies: %d\n", ret);
		return ret;
	}

	ret = tas5716_startup_sequence(tas5716);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to startup device\n");
		return ret;
	}

	/* CONFA */
	if (!pdata->thermal_warning_recovery)
		thermal |= TAS5716_CONFA_TWAB;
	if (!pdata->thermal_warning_adjustment)
		thermal |= TAS5716_CONFA_TWRB;
	if (!pdata->fault_detect_recovery)
		thermal |= TAS5716_CONFA_FDRB;
	regmap_update_bits(tas5716->regmap, TAS5716_CONFA,
			   TAS5716_CONFA_TWAB | TAS5716_CONFA_TWRB |
			   TAS5716_CONFA_FDRB,
			   thermal);

	/* CONFC */
	regmap_update_bits(tas5716->regmap, TAS5716_CONFC,
			   TAS5716_CONFC_OM_MASK,
			   pdata->ffx_power_output_mode
				<< TAS5716_CONFC_OM_SHIFT);
	regmap_update_bits(tas5716->regmap, TAS5716_CONFC,
			   TAS5716_CONFC_CSZ_MASK,
			   pdata->drop_compensation_ns
				<< TAS5716_CONFC_CSZ_SHIFT);
	regmap_update_bits(tas5716->regmap,
			   TAS5716_CONFC,
			   TAS5716_CONFC_OCRB,
			   pdata->oc_warning_adjustment ?
				TAS5716_CONFC_OCRB : 0);

	/* CONFE */
	regmap_update_bits(tas5716->regmap, TAS5716_CONFE,
			   TAS5716_CONFE_MPCV,
			   pdata->max_power_use_mpcc ?
				TAS5716_CONFE_MPCV : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_CONFE,
			   TAS5716_CONFE_MPC,
			   pdata->max_power_correction ?
				TAS5716_CONFE_MPC : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_CONFE,
			   TAS5716_CONFE_AME,
			   pdata->am_reduction_mode ?
				TAS5716_CONFE_AME : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_CONFE,
			   TAS5716_CONFE_PWMS,
			   pdata->odd_pwm_speed_mode ?
				TAS5716_CONFE_PWMS : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_CONFE,
			   TAS5716_CONFE_DCCV,
			   pdata->distortion_compensation ?
				TAS5716_CONFE_DCCV : 0);
	/*  CONFF */
	regmap_update_bits(tas5716->regmap, TAS5716_CONFF,
			   TAS5716_CONFF_IDE,
			   pdata->invalid_input_detect_mute ?
				TAS5716_CONFF_IDE : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_CONFF,
			   TAS5716_CONFF_OCFG_MASK,
			   pdata->output_conf
				<< TAS5716_CONFF_OCFG_SHIFT);

	/* channel to output mapping */
	regmap_update_bits(tas5716->regmap, TAS5716_C1CFG,
			   TAS5716_CxCFG_OM_MASK,
			   pdata->ch1_output_mapping
				<< TAS5716_CxCFG_OM_SHIFT);
	regmap_update_bits(tas5716->regmap, TAS5716_C2CFG,
			   TAS5716_CxCFG_OM_MASK,
			   pdata->ch2_output_mapping
				<< TAS5716_CxCFG_OM_SHIFT);
	regmap_update_bits(tas5716->regmap, TAS5716_C3CFG,
			   TAS5716_CxCFG_OM_MASK,
			   pdata->ch3_output_mapping
				<< TAS5716_CxCFG_OM_SHIFT);

	/* miscellaneous registers */
	regmap_update_bits(tas5716->regmap, TAS5716_MISC1,
			   TAS5716_MISC1_CPWMEN,
			   pdata->activate_mute_output ?
				TAS5716_MISC1_CPWMEN : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_MISC1,
			   TAS5716_MISC1_BRIDGOFF,
			   pdata->bridge_immediate_off ?
				TAS5716_MISC1_BRIDGOFF : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_MISC1,
			   TAS5716_MISC1_NSHHPEN,
			   pdata->noise_shape_dc_cut ?
				TAS5716_MISC1_NSHHPEN : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_MISC1,
			   TAS5716_MISC1_RPDNEN,
			   pdata->powerdown_master_vol ?
				TAS5716_MISC1_RPDNEN: 0);

	regmap_update_bits(tas5716->regmap, TAS5716_MISC2,
			   TAS5716_MISC2_PNDLSL_MASK,
			   pdata->powerdown_delay_divider
				<< TAS5716_MISC2_PNDLSL_SHIFT);

	/* initialize coefficient shadow RAM with reset values */
	for (i = 4; i <= 49; i += 5)
		tas5716->coef_shadow[i] = 0x400000;
	for (i = 50; i <= 54; i++)
		tas5716->coef_shadow[i] = 0x7fffff;
	tas5716->coef_shadow[55] = 0x5a9df7;
	tas5716->coef_shadow[56] = 0x7fffff;
	tas5716->coef_shadow[59] = 0x7fffff;
	tas5716->coef_shadow[60] = 0x400000;
	tas5716->coef_shadow[61] = 0x400000;

	tas5716_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	/* Bias level configuration will have done an extra enable */
	regulator_bulk_disable(ARRAY_SIZE(tas5716->supplies), tas5716->supplies);

	return 0;
}

static int tas5716_remove(struct snd_soc_codec *codec)
{
	struct tas5716_priv *tas5716 = snd_soc_codec_get_drvdata(codec);

	tas5716_set_bias_level(codec, SND_SOC_BIAS_OFF);
	regulator_bulk_disable(ARRAY_SIZE(tas5716->supplies), tas5716->supplies);

	return 0;
}

static const struct snd_soc_codec_driver tas5716_codec = {
	.probe =		tas5716_probe,
	.remove =		tas5716_remove,
	.suspend =		tas5716_suspend,
	.resume =		tas5716_resume,
	.set_bias_level =	tas5716_set_bias_level,
	.controls =		tas5716_snd_controls,
	.num_controls =		ARRAY_SIZE(tas5716_snd_controls),
	.dapm_widgets =		tas5716_dapm_widgets,
	.num_dapm_widgets =	ARRAY_SIZE(tas5716_dapm_widgets),
	.dapm_routes =		tas5716_dapm_routes,
	.num_dapm_routes =	ARRAY_SIZE(tas5716_dapm_routes),
};

static const struct regmap_config tas5716_regmap = {
	.reg_bits =		8,
	.val_bits =		8,
	.max_register =		TAS5716_MISC2,
	.reg_defaults =		tas5716_regs,
	.num_reg_defaults =	ARRAY_SIZE(tas5716_regs),
	.cache_type =		REGCACHE_RBTREE,
	.wr_table =		&tas5716_write_regs,
	.rd_table =		&tas5716_read_regs,
	.volatile_table =	&tas5716_volatile_regs,
};

static const struct of_device_id tas5716_dt_ids[] = {
	{ .compatible = "ti,tas5716", },
	{ }
};
MODULE_DEVICE_TABLE(of, tas5716_dt_ids);

static const char * const tas5716_ffx_modes[] = {
	[TAS5716_FFX_PM_DROP_COMP]		= "drop-compensation",
	[TAS5716_FFX_PM_TAPERED_COMP]		= "tapered-compensation",
	[TAS5716_FFX_PM_FULL_POWER]		= "full-power-mode",
	[TAS5716_FFX_PM_VARIABLE_DROP_COMP]	= "variable-drop-compensation",
};

static int tas5716_probe_dt(struct device *dev, struct tas5716_priv *tas5716)
{
	struct device_node *np = dev->of_node;
	struct tas5716_platform_data *pdata;
	const char *ffx_power_mode;
	u16 tmp;
	u8 tmp8;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	of_property_read_u8(np, "ti,output-conf",
			    &pdata->output_conf);
	of_property_read_u8(np, "ti,ch1-output-mapping",
			    &pdata->ch1_output_mapping);
	of_property_read_u8(np, "ti,ch2-output-mapping",
			    &pdata->ch2_output_mapping);
	of_property_read_u8(np, "ti,ch3-output-mapping",
			    &pdata->ch3_output_mapping);

	if (of_get_property(np, "ti,thermal-warning-recovery", NULL))
		pdata->thermal_warning_recovery = 1;
	if (of_get_property(np, "ti,thermal-warning-adjustment", NULL))
		pdata->thermal_warning_adjustment = 1;
	if (of_get_property(np, "ti,fault-detect-recovery", NULL))
		pdata->fault_detect_recovery = 1;

	pdata->ffx_power_output_mode = TAS5716_FFX_PM_VARIABLE_DROP_COMP;
	if (!of_property_read_string(np, "ti,ffx-power-output-mode",
				     &ffx_power_mode)) {
		int i, mode = -EINVAL;

		for (i = 0; i < ARRAY_SIZE(tas5716_ffx_modes); i++)
			if (!strcasecmp(ffx_power_mode, tas5716_ffx_modes[i]))
				mode = i;

		if (mode < 0)
			dev_warn(dev, "Unsupported ffx output mode: %s\n",
				 ffx_power_mode);
		else
			pdata->ffx_power_output_mode = mode;
	}

	tmp = 140;
	of_property_read_u16(np, "ti,drop-compensation-ns", &tmp);
	pdata->drop_compensation_ns = clamp_t(u16, tmp, 0, 300) / 20;

	if (of_get_property(np, "ti,overcurrent-warning-adjustment", NULL))
		pdata->oc_warning_adjustment = 1;

	/* CONFE */
	if (of_get_property(np, "ti,max-power-use-mpcc", NULL))
		pdata->max_power_use_mpcc = 1;

	if (of_get_property(np, "ti,max-power-correction", NULL))
		pdata->max_power_correction = 1;

	if (of_get_property(np, "ti,am-reduction-mode", NULL))
		pdata->am_reduction_mode = 1;

	if (of_get_property(np, "ti,odd-pwm-speed-mode", NULL))
		pdata->odd_pwm_speed_mode = 1;

	if (of_get_property(np, "ti,distortion-compensation", NULL))
		pdata->distortion_compensation = 1;

	/* CONFF */
	if (of_get_property(np, "ti,invalid-input-detect-mute", NULL))
		pdata->invalid_input_detect_mute = 1;

	/* MISC */
	if (of_get_property(np, "ti,activate-mute-output", NULL))
		pdata->activate_mute_output = 1;

	if (of_get_property(np, "ti,bridge-immediate-off", NULL))
		pdata->bridge_immediate_off = 1;

	if (of_get_property(np, "ti,noise-shape-dc-cut", NULL))
		pdata->noise_shape_dc_cut = 1;

	if (of_get_property(np, "ti,powerdown-master-volume", NULL))
		pdata->powerdown_master_vol = 1;

	if (!of_property_read_u8(np, "ti,powerdown-delay-divider", &tmp8)) {
		if (is_power_of_2(tmp8) && tmp8 >= 1 && tmp8 <= 128)
			pdata->powerdown_delay_divider = ilog2(tmp8);
		else
			dev_warn(dev, "Unsupported powerdown delay divider %d\n",
				 tmp8);
	}

	tas5716->pdata = pdata;

	return 0;
}


static int tas5716_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct device *dev = &i2c->dev;
	struct tas5716_priv *tas5716;
	int ret, i;

	printk("JDS - tas5716_i2c_probe\n");
	tas5716 = devm_kzalloc(dev, sizeof(struct tas5716_priv), GFP_KERNEL);
	if (!tas5716)
		return -ENOMEM;

	mutex_init(&tas5716->coeff_lock);
	tas5716->pdata = dev_get_platdata(dev);

	ret = tas5716_probe_dt(dev, tas5716);
	if (ret < 0)
		return ret;

	printk("JDS - tas5716_i2c_probe a\n");
	/* GPIOs */
	tas5716->gpiod_nreset = devm_gpiod_get(dev, "reset");
	if (IS_ERR(tas5716->gpiod_nreset)) {
		ret = PTR_ERR(tas5716->gpiod_nreset);
		if (ret != -ENOENT && ret != -ENOSYS)
			return ret;

		tas5716->gpiod_nreset = NULL;
	} else {
		gpiod_direction_output(tas5716->gpiod_nreset, 0);
	}

	printk("JDS - tas5716_i2c_probe b\n");
	tas5716->gpiod_power_down = devm_gpiod_get(dev, "power-down");
	if (IS_ERR(tas5716->gpiod_power_down)) {
		ret = PTR_ERR(tas5716->gpiod_power_down);
		if (ret != -ENOENT && ret != -ENOSYS)
			return ret;

		tas5716->gpiod_power_down = NULL;
	} else {
		gpiod_direction_output(tas5716->gpiod_power_down, 0);
	}

	/* regulators */
	for (i = 0; i < ARRAY_SIZE(tas5716->supplies); i++)
		tas5716->supplies[i].supply = tas5716_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(tas5716->supplies),
				      tas5716->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to request supplies: %d\n", ret);
		return ret;
	}

	printk("JDS - tas5716_i2c_probe c\n");
	tas5716->regmap = devm_regmap_init_i2c(i2c, &tas5716_regmap);
	if (IS_ERR(tas5716->regmap)) {
		ret = PTR_ERR(tas5716->regmap);
		dev_err(dev, "Failed to init regmap: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(i2c, tas5716);

	printk("JDS - tas5716_i2c_probe d\n");
	ret = snd_soc_register_codec(dev, &tas5716_codec, &tas5716_dai, 1);
	if (ret < 0)
		dev_err(dev, "Failed to register codec (%d)\n", ret);

	printk("JDS - tas5716_i2c_probe e\n");
	return ret;
}

static int tas5716_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct i2c_device_id tas5716_i2c_id[] = {
	{ "tas5716", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas5716_i2c_id);

static struct i2c_driver tas5716_i2c_driver = {
	.driver = {
		.name = "tas5716",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tas5716_dt_ids),
	},
	.probe =    tas5716_i2c_probe,
	.remove =   tas5716_i2c_remove,
	.id_table = tas5716_i2c_id,
};

module_i2c_driver(tas5716_i2c_driver);

MODULE_DESCRIPTION("ASoC TAS5716 driver");
MODULE_AUTHOR("Jon Smirl <jonsmirl@gmail.com>");
MODULE_LICENSE("GPL");
