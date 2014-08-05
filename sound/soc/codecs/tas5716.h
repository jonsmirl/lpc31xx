/*
 * Codec driver for TI TAS5716 2.1-channel high-efficiency digital audio system
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef _ASOC_TAS_5716_H
#define _ASOC_TAS_5716_H

/* TAS5716 register addresses */

#define TAS5716_CLOCK_CTRL	0x00 	// Clock control register
#define TAS5716_DEVICE_ID	0x01 	// Device ID register
#define TAS5716_ERROR		0x02	// Error status register
#define TAS5716_SYS_CTRL1	0x03	// System control 1 register
#define TAS5716_FORMAT		0x04 	// Serial-data interface register
#define TAS5716_SYS_CTRL2	0x05	// System-control 2 register
#define TAS5716_SOFT_MUTE	0x06	// Soft-mute register
#define TAS5716_MASTER_VOLUME	0x07	// Master volume
#define TAS5716_CH1_VOLUME	0x08	// Channel 1 volume
#define TAS5716_CH2_VOLUME	0x08	// Channel 2 volume
#define TAS5716_CH3_VOLUME	0x08	// Channel 3 volume
#define TAS5716_CH4_VOLUME	0x08	// Channel 4 volume
#define TAS5716_HP_VOLUME	0x0C	// HP volume
#define TAS5716_CH6_VOLUME	0x0D	// Channel-6 volume
#define TAS5716_VOLUME_CONFIG	0x0E 	// Volume configuration

#define TAS5716_MOD_LIMIT0	0x10	// Modulation-limit register
#define TAS5716_IC_DELAY_CH1	0x11 	// IC delay channel 1
#define TAS5716_IC_DELAY_CH2	0x12 	// IC delay channel 2
#define TAS5716_IC_DELAY_CH3	0x13 	// IC delay channel 3
#define TAS5716_IC_DELAY_CH4	0x14 	// IC delay channel 4
#define TAS5716_IC_DELAY_CH5	0x15 	// IC delay channel 5
#define TAS5716_IC_DELAY_CH6	0x16 	// IC delay channel 6
#define TAS5716_OFFSET		0x17	// Offset register

#define TAS5716_PWM_SHUTDOWN	0x19	// PWM shutdown group
#define TAS5716_START_STOP	0x1A	// Start/stop period
#define TAS5716_OSC_TRIM	0x1B	// Oscillator trim
#define TAS5716_BKND_ERR	0x1C	// BKND_ERR

#define TAS5716_INPUT_MUX	0x20	// Input MUX register
#define TAS5716_CH6_INPUT_MUX2	0x21	// Ch-6 input mux-2 register
#define TAS5716_AM_TUNED_FREQ	0x22	// AM tuned frequency

#define TAS5716_CH6_BQ_2	0x23	// ch6_bq[2] (loudness BQ)
#define TAS5716_CH6_BQ_3	0x24	// ch6_bq[3] (post volume)
#define TAS5716_PWM_MUX		0x25	// PWM MUX register
#define TAS5716_1_G		0x26	// 1/G register
#define TAS5716_SCALE		0x28	// Scale register
#define TAS5716_CH1_BQ_0	0x29	// ch1_bq[0]
#define TAS5716_CH1_BQ_1	0x2A	// ch1_bq[1]
#define TAS5716_CH1_BQ_2	0x2B	// ch1_bq[2]
#define TAS5716_CH1_BQ_3	0x2C	// ch1_bq[3]
#define TAS5716_CH1_BQ_4	0x2D	// ch1_bq[4]
#define TAS5716_CH1_BQ_5	0x2E	// ch1_bq[5]
#define TAS5716_CH1_BQ_6	0x2F	// ch1_bq[6]
#define TAS5716_CH2_BQ_0	0x30	// ch2_bq[0]
#define TAS5716_CH2_BQ_1	0x31	// ch2_bq[1]
#define TAS5716_CH2_BQ_2	0x32	// ch2_bq[2]
#define TAS5716_CH2_BQ_3	0x33	// ch2_bq[3]
#define TAS5716_CH2_BQ_4	0x34	// ch2_bq[4]
#define TAS5716_CH2_BQ_5	0x35	// ch2_bq[5]
#define TAS5716_CH2_BQ_6	0x36	// ch2_bq[6]
#define TAS5716_CH6_BQ_0	0x37	// ch6_bq[0]
#define TAS5716_CH6_BQ_1	0x38	// ch6_bq[1]

#define TAS5716_DRC1_AE		0x3A	// DRC1 ae
#define TAS5716_DRC1_AA		0x3B	// DRC1 aa
#define TAS5716_DRC1_AD		0x3C	// DRC1 ad
#define TAS5716_DRC2_AE		0x3F	// DRC2 ae
#define TAS5716_DRC2_AA		0x3E	// DRC2 aa
#define TAS5716_DRC2_AD		0x3F	// DRC2 ad
#define TAS5716_DRC1_T		0x40	// DRC1-T
#define TAS5716_DRC1_K		0x41	// DRC1-K
#define TAS5716_DRC1_O		0x42	// DRC1-O
#define TAS5716_DRC2_T		0x43	// DRC2-T
#define TAS5716_DRC2_K		0x44	// DRC2-K
#define TAS5716_DRC2_O		0x45	// DRC2-O
#define TAS5716_DRC_CONTROL	0x46	// DRC control

#define TAS5716_BANK_UPDATE	0x50	// Bank update command
#define TAS5716_V1OM		0x51	// V1OM3[31:0] V1OM1[31:0]
#define TAS5716_V2OM		0x52	// V2OM6[31:0] V2OM4[31:0] V2OM2[31:0]

/* TAS5716_CLOCK_CONTROL	0x00 	Clock control register */

#define TAS5716_CLOCK_CTRL_FS_MASK	(7<<5)
#define TAS5716_CLOCK_CTRL_32K		(0<<5)
#define TAS5716_CLOCK_CTRL_38K		(1<<5)
#define TAS5716_CLOCK_CTRL_441K		(2<<5)
#define TAS5716_CLOCK_CTRL_48K		(3<<5)
#define TAS5716_CLOCK_CTRL_882K		(4<<5)
#define TAS5716_CLOCK_CTRL_96K		(5<<5)
#define TAS5716_CLOCK_CTRL_176K		(6<<5)
#define TAS5716_CLOCK_CTRL_192K		(7<<5)
#define TAS5716_CLOCK_CTRL_FS_SHIFT	5
#define TAS5716_CLOCK_CTRL_FS_WIDTH	3
#define TAS5716_CLOCK_CTRL_MCLK_MASK	(7<<2)
#define TAS5716_CLOCK_CTRL_64FS		(0<<2)
#define TAS5716_CLOCK_CTRL_128FS	(1<<2)
#define TAS5716_CLOCK_CTRL_192FS	(2<<2)
#define TAS5716_CLOCK_CTRL_256FS	(3<<2)
#define TAS5716_CLOCK_CTRL_384FS	(4<<2)
#define TAS5716_CLOCK_CTRL_512FS	(5<<2)
#define TAS5716_CLOCK_CTRL_MCLK_SHIFT	2
#define TAS5716_CLOCK_CTRL_MCLK_WIDTH	3
#define TAS5716_CLOCK_CTRL_SCLK64	(0<<1)
#define TAS5716_CLOCK_CTRL_SCLK48	(1<<1)
#define TAS5716_CLOCK_CTRL_SCLK_MASK	(1<<1)
#define TAS5716_CLOCK_CTRL_SCLK_SHIFT	1
#define TAS5716_CLOCK_CTRL_SCLK_WIDTH	1
#define TAS5716_CLOCK_CTRL_VALID	(1<<0)
#define TAS5716_CLOCK_CTRL_VALID_MASK	(1<<0)
#define TAS5716_CLOCK_CTRL_VALID_SHIFT	0
#define TAS5716_CLOCK_CTRL_VALID_WIDTH	1

/* TAS5716_DEVICE_ID		0x01 	Device ID register */

/* TAS5716_ERROR_STATUS		0x02	Error status register */

#define TAS5716_ERROR_MCLK		(1<<7)
#define TAS5716_ERROR_MCLK_MASK		(1<<7)
#define TAS5716_ERROR_MCLK_SHIFT	7
#define TAS5716_ERROR_MCLK_WIDTH	1
#define TAS5716_ERROR_PLL		(1<<6)
#define TAS5716_ERROR_PLL_MASK		(1<<6)
#define TAS5716_ERROR_PLL_SHIFT		6
#define TAS5716_ERROR_PLL_WIDTH		1
#define TAS5716_ERROR_SCLK		(1<<5)
#define TAS5716_ERROR_SCLK_MASK		(1<<5)
#define TAS5716_ERROR_SCLK_SHIFT	5
#define TAS5716_ERROR_SCLK_WIDTH	1
#define TAS5716_ERROR_LRCLK		(1<<4)
#define TAS5716_ERROR_LRCLK_MASK	(1<<4)
#define TAS5716_ERROR_LRCLK_SHIFT	4
#define TAS5716_ERROR_LRCLK_WIDTH	1
#define TAS5716_ERROR_SLIP		(1<<3)
#define TAS5716_ERROR_SLIP_MASK		(1<<3)
#define TAS5716_ERROR_SLIP_SHIFT	3
#define TAS5716_ERROR_SLIP_WIDTH	1
#define TAS5716_ERROR_NONE		0

/* TAS5716_SYS_CTRL1		0x03	System control 1 register */

#define TAS5716_SYS_CTRL1_PWM_PASS	(1<<7)
#define TAS5716_SYS_CTRL1_PWM_MASK	(1<<7)
#define TAS5716_SYS_CTRL1_PWM_SHIFT	7
#define TAS5716_SYS_CTRL1_PWM_WIDTH	1
#define TAS5716_SYS_CTRL1_HARD_UNMUTE	(1<<5)
#define TAS5716_SYS_CTRL1_SOFT_UNMUTE	(0<<5)
#define TAS5716_SYS_CTRL1_UNMUTE_MASK	(1<<5)
#define TAS5716_SYS_CTRL1_UNMUTE_SHIFT	5
#define TAS5716_SYS_CTRL1_UNMUTE_WIDTH	1
#define TAS5716_SYS_CTRL1_AUTOCLK_OFF	(1<<3)
#define TAS5716_SYS_CTRL1_AUTOCLK_ON	(0<<3)
#define TAS5716_SYS_CTRL1_AUTOCLK_MASK	(1<<3)
#define TAS5716_SYS_CTRL1_AUTOCLK_SHIFT	3
#define TAS5716_SYS_CTRL1_AUTOCLK_WIDTH	1
#define TAS5716_SYS_CTRL1_SOFTSTART	(1<<2)
#define TAS5716_SYS_CTRL1_SOFT_MASK	(1<<2)
#define TAS5716_SYS_CTRL1_SOFT_SHIFT	2
#define TAS5716_SYS_CTRL1_SOFT_WIDTH	1
#define TAS5716_SYS_CTRL1_DEEMP_MASK	(3<<0)
#define TAS5716_SYS_CTRL1_DEEMP_NONE	(0<<0)
#define TAS5716_SYS_CTRL1_DEEMP_441	(2<<0)
#define TAS5716_SYS_CTRL1_DEEMP_48	(3<<0)
#define TAS5716_SYS_CTRL1_DEEMP_SHIFT	0
#define TAS5716_SYS_CTRL1_DEEMP_WIDTH	2

/* TAS5716_FORMAT		0x04 	Serial-data interface register */

#define TAS5716_FORMAT_RJ_16		0
#define TAS5716_FORMAT_RJ_20		1
#define TAS5716_FORMAT_RJ_24		2
#define TAS5716_FORMAT_I2S_16		3
#define TAS5716_FORMAT_I2S_20		4
#define TAS5716_FORMAT_I2S_24		5
#define TAS5716_FORMAT_LJ_16		6
#define TAS5716_FORMAT_LJ_20		7
#define TAS5716_FORMAT_LJ_24		8
#define TAS5716_FORMAT_RJ_18		0xA
#define TAS5716_FORMAT_I2S_16_32FS	0x13
#define TAS5716_FORMAT_LJ_16_32FS	0x16

/* TAS5716_SYS_CTRL2		0x05	System-control 2 register */

#define TAS5716_SYS_CTRL2_SHUT_MASK	(3<<5)
#define TAS5716_SYS_CTRL2_SHUTDOWN	(3<<5)
#define TAS5716_SYS_CTRL2_START_SDG	(0<<5)
#define TAS5716_SYS_CTRL2_START_ALL	(1<<5)
#define TAS5716_SYS_CTRL2_SHUT_SHIFT	5
#define TAS5716_SYS_CTRL2_SHUT_WIDTH	2
#define TAS5716_SYS_CTRL2_HPVOL		(1<<3)
#define TAS5716_SYS_CTRL2_CHVOL		(0<<3)
#define TAS5716_SYS_CTRL2_HPVOL_MASK	(1<<3)
#define TAS5716_SYS_CTRL2_HPVOL_SHIFT	3
#define TAS5716_SYS_CTRL2_HPVOL_WIDTH	1
#define TAS5716_SYS_CTRL2_SPK_MASK	(3<<1)
#define TAS5716_SYS_CTRL2_HPSEL		(0<<1)
#define TAS5716_SYS_CTRL2_HPMODE	(1<<1)
#define TAS5716_SYS_CTRL2_LINEOUT	(2<<1)
#define TAS5716_SYS_CTRL2_SPK_SHIFT	1
#define TAS5716_SYS_CTRL2_SPK_WIDTH	2

/* TAS5716_SOFT_MUTE		0x06	Soft-mute register */

#define TAS5716_SM_CH1			(1<<0)
#define TAS5716_SM_CH1_MASK		(1<<0)
#define TAS5716_SM_CH1_SHIFT		0
#define TAS5716_SM_CH1_WIDTH		1
#define TAS5716_SM_CH2			(1<<1)
#define TAS5716_SM_CH2_MASK		(1<<1)
#define TAS5716_SM_CH2_SHIFT		1
#define TAS5716_SM_CH2_WIDTH		1
#define TAS5716_SM_CH3			(1<<2)
#define TAS5716_SM_CH3_MASK		(1<<2)
#define TAS5716_SM_CH3_SHIFT		2
#define TAS5716_SM_CH3_WIDTH		1
#define TAS5716_SM_CH4			(1<<3)
#define TAS5716_SM_CH4_MASK		(1<<3)
#define TAS5716_SM_CH4_SHIFT		3
#define TAS5716_SM_CH4_WIDTH		1
#define TAS5716_SM_CH6			(1<<5)
#define TAS5716_SM_CH6_MASK		(1<<5)
#define TAS5716_SM_CH6_SHIFT		5
#define TAS5716_SM_CH6_WIDTH		1
#define TAS5716_SM_UNMUTE_ALL		0

/* TAS5716_MASTER_VOLUME	0x07	Master volume */
/* TAS5716_CH1_VOLUME		0x08	Channel 1 volume */
/* TAS5716_CH2_VOLUME		0x08	Channel 2 volume */
/* TAS5716_CH3_VOLUME		0x08	Channel 3 volume */
/* TAS5716_CH4_VOLUME		0x08	Channel 4 volume */
/* TAS5716_HP_VOLUME		0x0C	HP volume */
/* TAS5716_CH6_VOLUME		0x0D	Channel-6 volume */

/* TAS5716_VOLUME_CONFIG	0x0E 	Volume configuration */

#define TAS5716_VOLCONF_BIQUAD		(1<<6)
#define TAS5716_VOLCONF_BIQUAD_MASK	(1<<6)
#define TAS5716_VOLCONF_BIQUAD_SHIFT	6
#define TAS5716_VOLCONF_BIQUAD_WIDTH	1
#define TAS5716_VOLCONF_SLEW_MASK	(3<<0)
#define TAS5716_VOLCONF_SLEW512		(0<<0)
#define TAS5716_VOLCONF_SLEW1024	(1<<0)
#define TAS5716_VOLCONF_SLEW2048	(2<<0)
#define TAS5716_VOLCONF_SLEW256		(3<<0)
#define TAS5716_VOLCONF_SLEW_SHIFT	0
#define TAS5716_VOLCONF_SLEW_WIDTH	2

/* TAS5716_MOD_LIMIT0		0x10	Modulation-limit register */
/* TAS5716_IC_DELAY_CH1		0x11 	IC delay channel 1 */
/* TAS5716_IC_DELAY_CH2		0x12 	IC delay channel 2 */
/* TAS5716_IC_DELAY_CH3		0x13 	IC delay channel 3 */
/* TAS5716_IC_DELAY_CH4		0x14 	IC delay channel 4 */
/* TAS5716_IC_DELAY_CH5		0x15 	IC delay channel 5 */
/* TAS5716_IC_DELAY_CH6		0x16 	IC delay channel 6 */
/* TAS5716_OFFSET		0x17	Offset register */

/* TAS5716_PWM_SHUTDOWN		0x19	PWM shutdown group */

#define TAS5716_SDG_CH6			(1 << 5)
#define TAS5716_SDG_CH5			(1 << 4)
#define TAS5716_SDG_CH4			(1 << 3)
#define TAS5716_SDG_CH3			(1 << 2)
#define TAS5716_SDG_CH2			(1 << 1)
#define TAS5716_SDG_CH1			(1 << 0)

/* TAS5716_START_STOP		0x1A	Start/stop period */

#define TAS5716_SS_BTL			(1<<7)
#define TAS5716_SS_SE			(0<<7)
#define TAS5716_SS_LOAD_MASK		(1<<7)
#define TAS5716_SS_LOAD_SHIFT		6
#define TAS5716_SS_LOAD_WIDTH		1
#define TAS5716_SS_DUTY_MASK		(0x1F<<0)
#define TAS5716_SS_DUTY_SHIFT		0
#define TAS5716_SS_DUTY_WIDTH		5

/* TAS5716_OSC_TRIM		0x1B	Oscillator trim */
/* Write data 0x00 to register 0x1B (enable factory trim). */

#define TAS5716_OSC_DONE		(1<<7)
#define TAS5716_OSC_DONE_MASK		(1<<7)
#define TAS5716_OSC_DONE_SHIFT		7
#define TAS5716_OSC_DONE_WIDTH		1
#define TAS5716_OSC_NOTRIM		(1<<1)
#define TAS5716_OSC_NOTRIM_MASK		(1<<1)
#define TAS5716_OSC_NOTRIM_SHIFT	1
#define TAS5716_OSC_NOTRIM_WIDTH	1

/* TAS5716_BKND_ERR		0x1C	BKND_ERR */

/* TAS5716_INPUT_MUX		0x20	Input MUX register */
#define TAS5716_MUX_AD(ch)		0
#define TAS5716_MUX_BD(ch)		(8<<((6-ch)*4)))
#define TAS5716_MUX_IN1L(ch)		(0<<((6-ch)*4)))
#define TAS5716_MUX_IN1R(ch)		(1<<((6-ch)*4)))
#define TAS5716_MUX_IN2L(ch)		(2<<((6-ch)*4)))
#define TAS5716_MUX_IN2R(ch)		(3<<((6-ch)*4)))
#define TAS5716_MUX_BTL(ch)		(7<<((6-ch)*4)))

/* TAS5716_CH6_INPUT_MUX2	0x21	Ch-6 input mux-2 register */

#define TAS5716_MUX2_6_TO_6		(0<<8)
#define TAS5716_MUX2_6_BASS		(1<<8)
#define TAS5716_MUX2_LR_DIV_2		(2<<8)
#define TAS5716_MUX2_MASK		(3<<8)
#define TAS5716_MUX2_SHIFT		8
#define TAS5716_MUX2_WIDTH		2

/* TAS5716_AM_TUNED_FREQ	0x22	AM tuned frequency */

#define TAS5716_AM_MODE			(1<<20)
#define TAS5716_AM_MODE_MASK		(1<<20)
#define TAS5716_AM_MODE_SHIFT		20
#define TAS5716_AM_MODE_WIDTH		1
#define TAS5716_AM_SEQ_MASK		(3<<18)
#define TAS5716_AM_SEQ_SHIFT		18
#define TAS5716_AM_SEQ_WIDTH		1
#define TAS5716_AM_IF			(1<<17)
#define TAS5716_AM_IF_MASK		(1<<17)
#define TAS5716_AM_IF_SHIFT		17
#define TAS5716_AM_IF_WIDTH		1
#define TAS5716_AM_BCD			(1<<16)
#define TAS5716_AM_BCD_MASK		(1<<16)
#define TAS5716_AM_BCD_SHIFT		16
#define TAS5716_AM_BCD_WIDTH		1
#define TAS5716_AM_FREQ_MASK		(0xFFFF<<0)
#define TAS5716_AM_FREQ_SHIFT		0
#define TAS5716_AM_FREQ_WIDTH		16

/* TAS5716_PWM_MUX		0x25	PWM MUX register */
#define TAS5716_PWM_MUX_HPL(ch)		((ch-1)<<28)
#define TAS5716_PWM_MUX_HPR(ch)		((ch-1)<<24)
#define TAS5716_PWM_MUX_OUTA(ch)	((ch-1)<<20)
#define TAS5716_PWM_MUX_OUTB(ch)	((ch-1)<<16)
#define TAS5716_PWM_MUX_OUTC(ch)	((ch-1)<<12)
#define TAS5716_PWM_MUX_OUTD(ch)	((ch-1)<<8)
#define TAS5716_PWM_MUX_SUBM(ch)	((ch-1)<<4)
#define TAS5716_PWM_MUX_SUBP(ch)	((ch-1)<<0)

/* TAS5716_1_G			0x26	1/G register */
/* TAS5716_SCALE		0x28	Scale register */
/* TAS5716_CH1_BQ_0		0x29	ch1_bq[0] */
/* TAS5716_CH1_BQ_1		0x2A	ch1_bq[1] */
/* TAS5716_CH1_BQ_2		0x2B	ch1_bq[2] */
/* TAS5716_CH1_BQ_3		0x2C	ch1_bq[3] */
/* TAS5716_CH1_BQ_4		0x2D	ch1_bq[4] */
/* TAS5716_CH1_BQ_5		0x2E	ch1_bq[5] */
/* TAS5716_CH1_BQ_6		0x2F	ch1_bq[6] */
/* TAS5716_CH2_BQ_0		0x30	ch2_bq[0] */
/* TAS5716_CH2_BQ_1		0x31	ch2_bq[1] */
/* TAS5716_CH2_BQ_2		0x32	ch2_bq[2] */
/* TAS5716_CH2_BQ_3		0x33	ch2_bq[3] */
/* TAS5716_CH2_BQ_4		0x34	ch2_bq[4] */
/* TAS5716_CH2_BQ_5		0x35	ch2_bq[5] */
/* TAS5716_CH2_BQ_6		0x36	ch2_bq[6] */
/* TAS5716_CH6_BQ_0		0x37	ch6_bq[0] */
/* TAS5716_CH6_BQ_1		0x38	ch6_bq[1] */

/* TAS5716_DRC1_AE		0x3A	DRC1 ae */
/* TAS5716_DRC1_AA		0x3B	DRC1 aa */
/* TAS5716_DRC1_AD		0x3C	DRC1 ad */
/* TAS5716_DRC2_AE		0x3F	DRC2 ae */
/* TAS5716_DRC2_AA		0x3E	DRC2 aa */
/* TAS5716_DRC2_AD		0x3F	DRC2 ad */
/* TAS5716_DRC1_T		0x40	DRC1-T */
/* TAS5716_DRC1_K		0x41	DRC1-K */
/* TAS5716_DRC1_O		0x42	DRC1-O */
/* TAS5716_DRC2_T		0x43	DRC2-T */
/* TAS5716_DRC2_K		0x44	DRC2-K */
/* TAS5716_DRC2_O		0x45	DRC2-O */
/* TAS5716_DRC_CONTROL		0x46	DRC control */

#define TAS5716_DRC_CTRL_DRC1_CH4	(1 << 3)
#define TAS5716_DRC_CTRL_DRC1_CH3	(1 << 2)
#define TAS5716_DRC_CTRL_DRC2_SUB	(1 << 1)
#define TAS5716_DRC_CTRL_DRC2_SAT	(1 << 0)

/* TAS5716_BANK_UPDATE		0x50	Bank update command */

#define TAS5716_BANK_EQ_HEAD		(1 << 7)
#define TAS5716_BANK_DRC_HEAD		(1 << 6)
#define TAS5716_BANK_SW_MASK		(3 << 0)
#define TAS5716_BANK_SW_NONE		(0 << 0)
#define TAS5716_BANK_SW_BANK1		(1 << 0)
#define TAS5716_BANK_SW_BANK2		(2 << 0)
#define TAS5716_BANK_SW_BANK3		(3 << 0)
#define TAS5716_BANK_SW_AUTO		(4 << 0)

/* TAS5716_V1OM			0x51	V1OM3[31:0] V1OM1[31:0] */
/* TAS5716_V2OM			0x52	V2OM6[31:0] V2OM4[31:0] V2OM2[31:0] */

#endif /* _ASOC_TAS_5716_H */
