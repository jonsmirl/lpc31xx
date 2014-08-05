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
	{  0x0, 0x6c },
	{  0x1, 0x28 },
	{  0x2, 0x00 },
	{  0x3, 0xA0 },
	{  0x4, 0x05 },
	{  0x5, 0x40 },
	{  0x6, 0x00 },
	{  0x7, 0xff },
	{  0x8, 0x30 },
	{  0x9, 0x30 },
	{  0xa, 0x30 },
	{  0xb, 0x30 },
	{  0xc, 0x30 },
	{  0xd, 0x30 },
	{  0xe, 0x91 },
	{ 0x10, 0x02 },
	{ 0x11, 0x4c },
	{ 0x12, 0x34 },
	{ 0x13, 0x1c },
	{ 0x14, 0x64 },
	{ 0x15, 0xb0 },
	{ 0x16, 0x90 },
	{ 0x19, 0x30 },
	{ 0x1a, 0x0a },
	{ 0x1b, 0x82 },
	{ 0x1c, 0x02 },
	{ 0x20, 0x008977A },
	{ 0x21, 0x0004203 },
	{ 0x22, 0x00 },
	{ 0x25, 0x01021345 },
	{ 0x26, 0x00800000 },
	{ 0x28, 0x00800000 },
	{ 0x40, 0xFDA21490 },
	{ 0x41, 0x03842109 },
	{ 0x42, 0x00084210 },
	{ 0x43, 0xFDA21490 },
	{ 0x44, 0x03842109 },
	{ 0x45, 0x00084210 },
	{ 0x46, 0x00 },
	{ 0x50, 0x00 },
};

static const struct regmap_range tas5716_write_regs_range[] = {
	regmap_reg_range(TAS5716_SYS_CTRL1,  TAS5716_BKND_ERR),
	regmap_reg_range(TAS5716_INPUT_MUX,  TAS5716_AM_TUNED_FREQ),
	regmap_reg_range(TAS5716_PWM_MUX,  TAS5716_SCALE),
	regmap_reg_range(TAS5716_DRC1_T, TAS5716_BANK_UPDATE),
};

static const struct regmap_range tas5716_read_regs_range[] = {
	regmap_reg_range(TAS5716_SYS_CTRL1,  TAS5716_BKND_ERR),
	regmap_reg_range(TAS5716_INPUT_MUX,  TAS5716_AM_TUNED_FREQ),
	regmap_reg_range(TAS5716_PWM_MUX,  TAS5716_SCALE),
	regmap_reg_range(TAS5716_DRC1_T, TAS5716_BANK_UPDATE),
};

static const struct regmap_range tas5716_volatile_regs_range[] = {
	regmap_reg_range(TAS5716_CLOCK_CTRL, TAS5716_ERROR),
	regmap_reg_range(TAS5716_OSC_TRIM,  TAS5716_OSC_TRIM),
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
	"vdd",		/* digital supply, 3.3V */
	"vcc"		/* power amp supply, 5V - 26V */
};

/* codec private data */
struct tas5716_priv {
	struct regmap *regmap;
	struct regulator_bulk_data supplies[ARRAY_SIZE(tas5716_supply_names)];

	unsigned int mclk;
	unsigned int format;

	int shutdown;

	int gpio_nreset;
	int gpio_power_down;
	int gpio_mute;
	int gpio_hpsel;

	struct mutex coeff_lock;

	u8 system_control_1;
	u8 system_control_2;
	u8 soft_mute;
	u8 volume_configuration;
	u8 modulation_limit;
	u8 ic_delay_ch1;
	u8 ic_delay_ch2;
	u8 ic_delay_ch3;
	u8 ic_delay_ch4;
	u8 ic_delay_ch5;
	u8 ic_delay_ch6;
	u8 offset;
	u8 pwm_shutdown_group;
	u8 start_stop_period;
	u8 backend_error;
	u32 input_mux;
	u32 ch6_input_mux;
	u32 am_tuned_frequency;
	u32 pwm_mux;
	u32 drc_control;
	u32 bank_update;
};

static const DECLARE_TLV_DB_SCALE(vol_tlv, -10000, 48, 1);


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
#ifdef jds
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
#endif

/* Byte lengths of the variable length registers in the TAS5716 */
static int tas5716_register_size(int reg) {
	switch (reg) {
	case 0 ... 0x1f:
		return 1;
	case 0x20 ... 0x22:
		return 2;
	case 0x23 ... 0x24:
		return 20;
	case 0x25 ... 0x26:
		return 4;
	case 0x27:
		return 1;
	case 0x28:
		return 4;
	case 0x29 ... 0x38:
		return 20;
	case 0x39:
		return 4;
	case 0x3a ... 0x3f:
		return 8;
	case 0x40 ... 0x50:
		return 4;
	case 0x51:
		return 8;
	case 0x52:
		return 12;
	case 0x53 ... 0xFF:
		return 4;
	default:
		return -EINVAL;
	}
}

static int tas5716_reg_write(void *context, unsigned int reg, unsigned int value)
{
	struct i2c_client *client = context;
	unsigned int i;
	int size;
	uint8_t buf[5];
	int ret;

	size = tas5716_register_size(reg);
	if (size < 0)
		return size;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	for (i = size + 1; i >= 2; --i) {
		buf[i] = value;
		value >>= 8;
	}

	ret = i2c_master_send(client, buf, size + 2);
	if (ret == size + 2)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int tas5716_reg_read(void *context, unsigned int reg, unsigned int *value)
{
	int ret;
	unsigned int i;
	int size;
	uint8_t send_buf[2], recv_buf[3];
	struct i2c_client *client = context;
	struct i2c_msg msgs[2];

	size = tas5716_register_size(reg);
	if (size < 0)
		return size;

	send_buf[0] = reg >> 8;
	send_buf[1] = reg & 0xff;

	msgs[0].addr = client->addr;
	msgs[0].len = sizeof(send_buf);
	msgs[0].buf = send_buf;
	msgs[0].flags = 0;

	msgs[1].addr = client->addr;
	msgs[1].len = size;
	msgs[1].buf = recv_buf;
	msgs[1].flags = I2C_M_RD;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	else if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*value = 0;

	for (i = 0; i < size; i++)
		*value |= recv_buf[i] << (i * 8);

	return 0;
}


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
	uinfo->count = numcoef;
	return 0;
}

static int tas5716_coefficient_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
#ifdef jds
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
#endif
	return 0;
}

static int tas5716_coefficient_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
#ifdef jds
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
	for (i = 0; i < 3 * numcoef; i++)
		regmap_write(tas5716->regmap, TAS5716_B1CF1 + i,
			     ucontrol->value.bytes.data[i]);
	if (numcoef == 1)
		regmap_write(tas5716->regmap, TAS5716_CFUD, cfud | 0x01);
	else if (numcoef == 5)
		regmap_write(tas5716->regmap, TAS5716_CFUD, cfud | 0x02);
	else
		return -EINVAL;
#endif
	return 0;
}

static int tas5716_cache_sync(struct snd_soc_codec *codec)
{
#ifdef jds
	struct tas5716_priv *tas5716 = snd_soc_codec_get_drvdata(codec);
	unsigned int mute;
	int rc;

	/* mute during register sync */
	regmap_read(tas5716->regmap, TAS5716_CFUD, &mute);
	regmap_write(tas5716->regmap, TAS5716_MMUTE, mute | TAS5716_MMUTE_MMUTE);
	rc = regcache_sync(tas5716->regmap);
	regmap_write(tas5716->regmap, TAS5716_MMUTE, mute);

	return rc;
#endif
	return 0;
}

#define COEFS_8(xname, index) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = tas5716_coefficient_info, \
	.get = tas5716_coefficient_get,\
	.put = tas5716_coefficient_put, \
	.private_value = index | (8 << 16) }

#define COEFS_12(xname, index) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = tas5716_coefficient_info, \
	.get = tas5716_coefficient_get,\
	.put = tas5716_coefficient_put, \
	.private_value = index | (12 << 16) }

#define COEFS_20(xname, index) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = tas5716_coefficient_info, \
	.get = tas5716_coefficient_get,\
	.put = tas5716_coefficient_put, \
	.private_value = index | (20 << 16) }

static const struct snd_kcontrol_new tas5716_snd_controls[] = {
SOC_SINGLE_TLV("Master Volume", TAS5716_MASTER_VOLUME, 0, 0xff, 1, vol_tlv),
SOC_SINGLE_TLV("Channel 1 Volume", TAS5716_CH1_VOLUME, 0, 0xff, 1, vol_tlv),
SOC_SINGLE_TLV("Channel 2 Volume", TAS5716_CH2_VOLUME, 0, 0xff, 1, vol_tlv),
SOC_SINGLE_TLV("Channel 3 Volume", TAS5716_CH3_VOLUME, 0, 0xff, 1, vol_tlv),
SOC_SINGLE_TLV("Channel 4 Volume", TAS5716_CH4_VOLUME, 0, 0xff, 1, vol_tlv),
SOC_SINGLE_TLV("HP Volume", TAS5716_HP_VOLUME, 0, 0xff, 1, vol_tlv),
SOC_SINGLE_TLV("Channel 5 Volume", TAS5716_CH6_VOLUME, 0, 0xff, 1, vol_tlv),



#ifdef JDS
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
#endif

COEFS_20("ch6_bq[2] (loudness BQ)", 0x23),
COEFS_20("ch6_bq[3] (post volume BQ)", 0x24),
COEFS_20("ch1_bq[0]", 0x29),
COEFS_20("ch1_bq[1]", 0x2a),
COEFS_20("ch1_bq[2]", 0x2b),
COEFS_20("ch1_bq[3]", 0x2c),
COEFS_20("ch1_bq[4]", 0x2d),
COEFS_20("ch1_bq[5]", 0x2e),
COEFS_20("ch1_bq[6]", 0x2f),
COEFS_20("ch2_bq[0]", 0x30),
COEFS_20("ch2_bq[1]", 0x31),
COEFS_20("ch2_bq[2]", 0x32),
COEFS_20("ch2_bq[3]", 0x33),
COEFS_20("ch2_bq[4]", 0x34),
COEFS_20("ch2_bq[5]", 0x35),
COEFS_20("ch2_bq[6]", 0x36),
COEFS_20("ch6_bq[0]", 0x37),
COEFS_20("ch6_bq[1]", 0x38),

COEFS_8("DRC1 ae", 0x3a),
COEFS_8("DRC1 aa", 0x3b),
COEFS_8("DRC1 ad", 0x3c),
COEFS_8("DRC1 ae", 0x3d),
COEFS_8("DRC2 aa", 0x3e),
COEFS_8("DRC2 ad", 0x3f),

COEFS_8("V1OM", 0x51),
COEFS_12("V2OM", 0x52),
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
//	unsigned int confb = 0;

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
//		confb |= TAS5716_CONFB_C2IM;
		break;
	case SND_SOC_DAIFMT_NB_IF:
//		confb |= TAS5716_CONFB_C1IM;
		break;
	default:
		return -EINVAL;
	}
#ifdef jds
	return regmap_update_bits(tas5716->regmap, TAS5716_CONFB,
				  TAS5716_CONFB_C1IM | TAS5716_CONFB_C2IM, confb);
#endif
	return 0;
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
//	unsigned int confa;
	unsigned int confb;
	unsigned int rate, ratio;
//	int ret;

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

//	confa = (ir << TAS5716_CONFA_IR_SHIFT) |
//		(mcs << TAS5716_CONFA_MCS_SHIFT);
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
#ifdef jds
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
#endif
	return 0;
}

static int tas5716_startup_sequence(struct tas5716_priv *tas5716)
{
	if (tas5716->gpio_power_down)
		gpio_set_value(tas5716->gpio_power_down, 1);

	if (tas5716->gpio_nreset) {
		gpio_set_value(tas5716->gpio_nreset, 0);
		mdelay(1);
		gpio_set_value(tas5716->gpio_nreset, 1);
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
#ifdef jds
		regmap_update_bits(tas5716->regmap, TAS5716_CONFF,
				   TAS5716_CONFF_PWDN | TAS5716_CONFF_EAPD,
				   TAS5716_CONFF_PWDN | TAS5716_CONFF_EAPD);
#endif
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
#ifdef JDS
		regmap_update_bits(tas5716->regmap, TAS5716_CONFF,
				   TAS5716_CONFF_PWDN | TAS5716_CONFF_EAPD,
				   0);
#endif

		break;

	case SND_SOC_BIAS_OFF:
		/* The chip runs through the power down sequence for us */
#ifdef JDS
		regmap_update_bits(tas5716->regmap, TAS5716_CONFF,
				   TAS5716_CONFF_PWDN | TAS5716_CONFF_EAPD, 0);
#endif

		/* power down: low */
		if (tas5716->gpio_power_down)
			gpio_set_value(tas5716->gpio_power_down, 0);

		if (tas5716->gpio_nreset)
			gpio_set_value(tas5716->gpio_nreset, 0);

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
	int ret = 0;
//	int thermal = 0;

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
#ifdef jds
	/* CONFA */
	if (!tas5716->thermal_warning_recovery)
		thermal |= TAS5716_CONFA_TWAB;
	if (!tas5716->thermal_warning_adjustment)
		thermal |= TAS5716_CONFA_TWRB;
	if (!tas5716->fault_detect_recovery)
		thermal |= TAS5716_CONFA_FDRB;
	regmap_update_bits(tas5716->regmap, TAS5716_CONFA,
			   TAS5716_CONFA_TWAB | TAS5716_CONFA_TWRB |
			   TAS5716_CONFA_FDRB,
			   thermal);

	/* CONFC */
	regmap_update_bits(tas5716->regmap, TAS5716_CONFC,
			   TAS5716_CONFC_OM_MASK,
			   tas5716->ffx_power_output_mode
				<< TAS5716_CONFC_OM_SHIFT);
	regmap_update_bits(tas5716->regmap, TAS5716_CONFC,
			   TAS5716_CONFC_CSZ_MASK,
			   tas5716->drop_compensation_ns
				<< TAS5716_CONFC_CSZ_SHIFT);
	regmap_update_bits(tas5716->regmap,
			   TAS5716_CONFC,
			   TAS5716_CONFC_OCRB,
			   tas5716->oc_warning_adjustment ?
				TAS5716_CONFC_OCRB : 0);

	/* CONFE */
	regmap_update_bits(tas5716->regmap, TAS5716_CONFE,
			   TAS5716_CONFE_MPCV,
			   tas5716->max_power_use_mpcc ?
				TAS5716_CONFE_MPCV : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_CONFE,
			   TAS5716_CONFE_MPC,
			   tas5716->max_power_correction ?
				TAS5716_CONFE_MPC : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_CONFE,
			   TAS5716_CONFE_AME,
			   tas5716->am_reduction_mode ?
				TAS5716_CONFE_AME : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_CONFE,
			   TAS5716_CONFE_PWMS,
			   tas5716->odd_pwm_speed_mode ?
				TAS5716_CONFE_PWMS : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_CONFE,
			   TAS5716_CONFE_DCCV,
			   tas5716->distortion_compensation ?
				TAS5716_CONFE_DCCV : 0);
	/*  CONFF */
	regmap_update_bits(tas5716->regmap, TAS5716_CONFF,
			   TAS5716_CONFF_IDE,
			   tas5716->invalid_input_detect_mute ?
				TAS5716_CONFF_IDE : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_CONFF,
			   TAS5716_CONFF_OCFG_MASK,
			   tas5716->output_conf
				<< TAS5716_CONFF_OCFG_SHIFT);

	/* channel to output mapping */
	regmap_update_bits(tas5716->regmap, TAS5716_C1CFG,
			   TAS5716_CxCFG_OM_MASK,
			   tas5716->ch1_output_mapping
				<< TAS5716_CxCFG_OM_SHIFT);
	regmap_update_bits(tas5716->regmap, TAS5716_C2CFG,
			   TAS5716_CxCFG_OM_MASK,
			   tas5716->ch2_output_mapping
				<< TAS5716_CxCFG_OM_SHIFT);
	regmap_update_bits(tas5716->regmap, TAS5716_C3CFG,
			   TAS5716_CxCFG_OM_MASK,
			   tas5716->ch3_output_mapping
				<< TAS5716_CxCFG_OM_SHIFT);

	/* miscellaneous registers */
	regmap_update_bits(tas5716->regmap, TAS5716_MISC1,
			   TAS5716_MISC1_CPWMEN,
			   tas5716->activate_mute_output ?
				TAS5716_MISC1_CPWMEN : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_MISC1,
			   TAS5716_MISC1_BRIDGOFF,
			   tas5716->bridge_immediate_off ?
				TAS5716_MISC1_BRIDGOFF : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_MISC1,
			   TAS5716_MISC1_NSHHPEN,
			   tas5716->noise_shape_dc_cut ?
				TAS5716_MISC1_NSHHPEN : 0);
	regmap_update_bits(tas5716->regmap, TAS5716_MISC1,
			   TAS5716_MISC1_RPDNEN,
			   tas5716->powerdown_master_vol ?
				TAS5716_MISC1_RPDNEN: 0);

	regmap_update_bits(tas5716->regmap, TAS5716_MISC2,
			   TAS5716_MISC2_PNDLSL_MASK,
			   tas5716->powerdown_delay_divider
				<< TAS5716_MISC2_PNDLSL_SHIFT);
#endif
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
	.val_bits =		32,
	.max_register =		TAS5716_V2OM,
	.reg_defaults =		tas5716_regs,
	.num_reg_defaults =	ARRAY_SIZE(tas5716_regs),
	.cache_type =		REGCACHE_RBTREE,
	.volatile_table =	&tas5716_volatile_regs,
	.wr_table =		&tas5716_write_regs,
	.rd_table =		&tas5716_read_regs,
	.reg_write =		tas5716_reg_write,
	.reg_read =		tas5716_reg_read,
};

static const struct of_device_id tas5716_dt_ids[] = {
	{ .compatible = "ti,tas5716", },
	{ }
};
MODULE_DEVICE_TABLE(of, tas5716_dt_ids);

#ifdef jds
static const char * const tas5716_ffx_modes[] = {
	[TAS5716_FFX_PM_DROP_COMP]		= "drop-compensation",
	[TAS5716_FFX_PM_TAPERED_COMP]		= "tapered-compensation",
	[TAS5716_FFX_PM_FULL_POWER]		= "full-power-mode",
	[TAS5716_FFX_PM_VARIABLE_DROP_COMP]	= "variable-drop-compensation",
};
#endif

static int tas5716_probe_dt(struct device *dev, struct tas5716_priv *tas5716)
{
	struct device_node *np = dev->of_node;

	of_property_read_u8(np, "ti,system-control-1", &tas5716->system_control_1);
	of_property_read_u8(np, "ti,system-control-2", &tas5716->system_control_2);
	of_property_read_u8(np, "ti,soft-mute", &tas5716->soft_mute);
	of_property_read_u8(np, "ti,volume-configuration", &tas5716->volume_configuration);
	of_property_read_u8(np, "ti,modulation-limit", &tas5716->modulation_limit);
	of_property_read_u8(np, "ti,ic-delay-ch1", &tas5716->ic_delay_ch1);
	of_property_read_u8(np, "ti,ic-delay-ch2", &tas5716->ic_delay_ch2);
	of_property_read_u8(np, "ti,ic-delay-ch3", &tas5716->ic_delay_ch3);
	of_property_read_u8(np, "ti,ic-delay-ch4", &tas5716->ic_delay_ch4);
	of_property_read_u8(np, "ti,ic-delay-ch5", &tas5716->ic_delay_ch5);
	of_property_read_u8(np, "ti,ic-delay-ch6", &tas5716->ic_delay_ch6);
	of_property_read_u8(np, "ti,offset", &tas5716->offset);
	of_property_read_u8(np, "ti,pwm-shutdown-group", &tas5716->pwm_shutdown_group);
	of_property_read_u8(np, "ti,start-stop-period", &tas5716->start_stop_period);
	of_property_read_u8(np, "ti,backend-error", &tas5716->backend_error);
	of_property_read_u32(np, "ti,input-mux", &tas5716->input_mux);
	of_property_read_u32(np, "ti,ch6-input-mux", &tas5716->ch6_input_mux);
	of_property_read_u32(np, "ti,am-tuned-frequency", &tas5716->am_tuned_frequency);
	of_property_read_u32(np, "ti,pwm-mux", &tas5716->pwm_mux);
	of_property_read_u32(np, "ti,drc-control", &tas5716->drc_control);
	of_property_read_u32(np, "ti,bank-update", &tas5716->bank_update);

	return 0;
}


static int tas5716_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	enum of_gpio_flags flags;
	struct device *dev = &i2c->dev;
	struct device_node *np = dev->of_node;
	struct tas5716_priv *tas5716;
	int ret, i;

	printk("JDS - tas5716_i2c_probe\n");
	tas5716 = devm_kzalloc(dev, sizeof(struct tas5716_priv), GFP_KERNEL);
	if (!tas5716)
		return -ENOMEM;

	mutex_init(&tas5716->coeff_lock);

	ret = tas5716_probe_dt(dev, tas5716);
	if (ret < 0)
		return ret;

	printk("JDS - tas5716_i2c_probe a\n");

	/* GPIOs */
	tas5716->gpio_nreset = of_get_named_gpio_flags(np, "reset", 0, &flags);
	if (gpio_is_valid(tas5716->gpio_nreset)) {
		ret = devm_gpio_request_one(dev, tas5716->gpio_nreset,
			     flags & OF_GPIO_ACTIVE_LOW ?
				GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH,
			     "tas5716:reset");
		if (ret < 0)
			return ret;
	}

	tas5716->gpio_power_down = of_get_named_gpio_flags(np, "power-down", 0, &flags);
	if (gpio_is_valid(tas5716->gpio_power_down)) {
		ret = devm_gpio_request_one(dev, tas5716->gpio_power_down,
			     flags & OF_GPIO_ACTIVE_LOW ?
				GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH,
			     "tas5716:power-down");
		if (ret < 0)
			return ret;
	}

	tas5716->gpio_mute = of_get_named_gpio_flags(np, "mute", 0, &flags);
	if (gpio_is_valid(tas5716->gpio_mute)) {
		ret = devm_gpio_request_one(dev, tas5716->gpio_mute,
			     flags & OF_GPIO_ACTIVE_LOW ?
				GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH,
			     "tas5716:mute");
		if (ret < 0)
			return ret;
	}

	tas5716->gpio_hpsel = of_get_named_gpio_flags(np, "ti,hpsel", 0, &flags);
	if (gpio_is_valid(tas5716->gpio_hpsel)) {
		ret = devm_gpio_request_one(dev, tas5716->gpio_hpsel,
			     flags & OF_GPIO_ACTIVE_LOW ?
				GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH,
			     "tas5716:ti,hpsel");
		if (ret < 0)
			return ret;
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
