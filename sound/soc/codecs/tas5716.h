/*
 * Codec driver for ST TAS5716 2.1-channel high-efficiency digital audio system
 *
 * Copyright: 2011 Raumfeld GmbH
 * Author: Sven Brandau <info@brandau.biz>
 *
 * based on code from:
 *      Raumfeld GmbH
 *        Johannes Stezenbach <js@sig21.net>
 *	Wolfson Microelectronics PLC.
 *	  Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef _ASOC_TAS_5716_H
#define _ASOC_TAS_5716_H

/* STA50 register addresses */

#define TAS5716_REGISTER_COUNT	0x4D
#define TAS5716_COEF_COUNT 62

#define TAS5716_CONFA	0x00
#define TAS5716_CONFB    0x01
#define TAS5716_CONFC    0x02
#define TAS5716_CONFD    0x03
#define TAS5716_CONFE    0x04
#define TAS5716_CONFF    0x05
#define TAS5716_MMUTE    0x06
#define TAS5716_MVOL     0x07
#define TAS5716_C1VOL    0x08
#define TAS5716_C2VOL    0x09
#define TAS5716_C3VOL    0x0a
#define TAS5716_AUTO1    0x0b
#define TAS5716_AUTO2    0x0c
#define TAS5716_AUTO3    0x0d
#define TAS5716_C1CFG    0x0e
#define TAS5716_C2CFG    0x0f
#define TAS5716_C3CFG    0x10
#define TAS5716_TONE     0x11
#define TAS5716_L1AR     0x12
#define TAS5716_L1ATRT   0x13
#define TAS5716_L2AR     0x14
#define TAS5716_L2ATRT   0x15
#define TAS5716_CFADDR2  0x16
#define TAS5716_B1CF1    0x17
#define TAS5716_B1CF2    0x18
#define TAS5716_B1CF3    0x19
#define TAS5716_B2CF1    0x1a
#define TAS5716_B2CF2    0x1b
#define TAS5716_B2CF3    0x1c
#define TAS5716_A1CF1    0x1d
#define TAS5716_A1CF2    0x1e
#define TAS5716_A1CF3    0x1f
#define TAS5716_A2CF1    0x20
#define TAS5716_A2CF2    0x21
#define TAS5716_A2CF3    0x22
#define TAS5716_B0CF1    0x23
#define TAS5716_B0CF2    0x24
#define TAS5716_B0CF3    0x25
#define TAS5716_CFUD     0x26
#define TAS5716_MPCC1    0x27
#define TAS5716_MPCC2    0x28
#define TAS5716_DCC1     0x29
#define TAS5716_DCC2     0x2a
#define TAS5716_FDRC1    0x2b
#define TAS5716_FDRC2    0x2c
#define TAS5716_STATUS   0x2d
/* reserved: 0x2d - 0x30 */
#define TAS5716_EQCFG    0x31
#define TAS5716_EATH1    0x32
#define TAS5716_ERTH1    0x33
#define TAS5716_EATH2    0x34
#define TAS5716_ERTH2    0x35
#define TAS5716_CONFX    0x36
#define TAS5716_SVCA     0x37
#define TAS5716_SVCB     0x38
#define TAS5716_RMS0A    0x39
#define TAS5716_RMS0B    0x3a
#define TAS5716_RMS0C    0x3b
#define TAS5716_RMS1A    0x3c
#define TAS5716_RMS1B    0x3d
#define TAS5716_RMS1C    0x3e
#define TAS5716_EVOLRES  0x3f
/* reserved: 0x40 - 0x47 */
#define TAS5716_NSHAPE   0x48
#define TAS5716_CTXB4B1  0x49
#define TAS5716_CTXB7B5  0x4a
#define TAS5716_MISC1    0x4b
#define TAS5716_MISC2    0x4c

/* 0x00 CONFA */
#define TAS5716_CONFA_MCS_MASK	0x03
#define TAS5716_CONFA_MCS_SHIFT	0
#define TAS5716_CONFA_IR_MASK	0x18
#define TAS5716_CONFA_IR_SHIFT	3
#define TAS5716_CONFA_TWRB	BIT(5)
#define TAS5716_CONFA_TWAB	BIT(6)
#define TAS5716_CONFA_FDRB	BIT(7)

/* 0x01 CONFB */
#define TAS5716_CONFB_SAI_MASK	0x0f
#define TAS5716_CONFB_SAI_SHIFT	0
#define TAS5716_CONFB_SAIFB	BIT(4)
#define TAS5716_CONFB_DSCKE	BIT(5)
#define TAS5716_CONFB_C1IM	BIT(6)
#define TAS5716_CONFB_C2IM	BIT(7)

/* 0x02 CONFC */
#define TAS5716_CONFC_OM_MASK	0x03
#define TAS5716_CONFC_OM_SHIFT	0
#define TAS5716_CONFC_CSZ_MASK	0x3c
#define TAS5716_CONFC_CSZ_SHIFT	2
#define TAS5716_CONFC_OCRB	BIT(7)

/* 0x03 CONFD */
#define TAS5716_CONFD_HPB_SHIFT	0
#define TAS5716_CONFD_DEMP_SHIFT	1
#define TAS5716_CONFD_DSPB_SHIFT	2
#define TAS5716_CONFD_PSL_SHIFT	3
#define TAS5716_CONFD_BQL_SHIFT	4
#define TAS5716_CONFD_DRC_SHIFT	5
#define TAS5716_CONFD_ZDE_SHIFT	6
#define TAS5716_CONFD_SME_SHIFT	7

/* 0x04 CONFE */
#define TAS5716_CONFE_MPCV	BIT(0)
#define TAS5716_CONFE_MPCV_SHIFT	0
#define TAS5716_CONFE_MPC	BIT(1)
#define TAS5716_CONFE_MPC_SHIFT	1
#define TAS5716_CONFE_NSBW	BIT(2)
#define TAS5716_CONFE_NSBW_SHIFT	2
#define TAS5716_CONFE_AME	BIT(3)
#define TAS5716_CONFE_AME_SHIFT	3
#define TAS5716_CONFE_PWMS	BIT(4)
#define TAS5716_CONFE_PWMS_SHIFT	4
#define TAS5716_CONFE_DCCV	BIT(5)
#define TAS5716_CONFE_DCCV_SHIFT	5
#define TAS5716_CONFE_ZCE	BIT(6)
#define TAS5716_CONFE_ZCE_SHIFT	6
#define TAS5716_CONFE_SVE	BIT(7)
#define TAS5716_CONFE_SVE_SHIFT	7

/* 0x05 CONFF */
#define TAS5716_CONFF_OCFG_MASK	0x03
#define TAS5716_CONFF_OCFG_SHIFT	0
#define TAS5716_CONFF_IDE	BIT(2)
#define TAS5716_CONFF_BCLE	BIT(3)
#define TAS5716_CONFF_LDTE	BIT(4)
#define TAS5716_CONFF_ECLE	BIT(5)
#define TAS5716_CONFF_PWDN	BIT(6)
#define TAS5716_CONFF_EAPD	BIT(7)

/* 0x06 MMUTE */
#define TAS5716_MMUTE_MMUTE		0x01
#define TAS5716_MMUTE_MMUTE_SHIFT	0
#define TAS5716_MMUTE_C1M		0x02
#define TAS5716_MMUTE_C1M_SHIFT		1
#define TAS5716_MMUTE_C2M		0x04
#define TAS5716_MMUTE_C2M_SHIFT		2
#define TAS5716_MMUTE_C3M		0x08
#define TAS5716_MMUTE_C3M_SHIFT		3
#define TAS5716_MMUTE_LOC_MASK		0xC0
#define TAS5716_MMUTE_LOC_SHIFT		6

/* 0x0b AUTO1 */
#define TAS5716_AUTO1_AMGC_MASK	0x30
#define TAS5716_AUTO1_AMGC_SHIFT	4

/* 0x0c AUTO2 */
#define TAS5716_AUTO2_AMAME	0x01
#define TAS5716_AUTO2_AMAM_MASK	0x0e
#define TAS5716_AUTO2_AMAM_SHIFT	1
#define TAS5716_AUTO2_XO_MASK	0xf0
#define TAS5716_AUTO2_XO_SHIFT	4

/* 0x0d AUTO3 */
#define TAS5716_AUTO3_PEQ_MASK	0x1f
#define TAS5716_AUTO3_PEQ_SHIFT	0

/* 0x0e 0x0f 0x10 CxCFG */
#define TAS5716_CxCFG_TCB_SHIFT	0
#define TAS5716_CxCFG_EQBP_SHIFT	1
#define TAS5716_CxCFG_VBP_SHIFT	2
#define TAS5716_CxCFG_BO_SHIFT	3
#define TAS5716_CxCFG_LS_SHIFT	4
#define TAS5716_CxCFG_OM_MASK	0xc0
#define TAS5716_CxCFG_OM_SHIFT	6

/* 0x11 TONE */
#define TAS5716_TONE_BTC_SHIFT	0
#define TAS5716_TONE_TTC_SHIFT	4

/* 0x12 0x13 0x14 0x15 limiter attack/release */
#define TAS5716_LxA_SHIFT	0
#define TAS5716_LxR_SHIFT	4

/* 0x26 CFUD */
#define TAS5716_CFUD_W1		0x01
#define TAS5716_CFUD_WA		0x02
#define TAS5716_CFUD_R1		0x04
#define TAS5716_CFUD_RA		0x08


/* biquad filter coefficient table offsets */
#define TAS5716_C1_BQ_BASE	0
#define TAS5716_C2_BQ_BASE	20
#define TAS5716_CH_BQ_NUM	4
#define TAS5716_BQ_NUM_COEF	5
#define TAS5716_XO_HP_BQ_BASE	40
#define TAS5716_XO_LP_BQ_BASE	45
#define TAS5716_C1_PRESCALE	50
#define TAS5716_C2_PRESCALE	51
#define TAS5716_C1_POSTSCALE	52
#define TAS5716_C2_POSTSCALE	53
#define TAS5716_C3_POSTSCALE	54
#define TAS5716_TW_POSTSCALE	55
#define TAS5716_C1_MIX1		56
#define TAS5716_C1_MIX2		57
#define TAS5716_C2_MIX1		58
#define TAS5716_C2_MIX2		59
#define TAS5716_C3_MIX1		60
#define TAS5716_C3_MIX2		61

/* miscellaneous register 1 */
#define TAS5716_MISC1_CPWMEN	BIT(2)
#define TAS5716_MISC1_BRIDGOFF	BIT(5)
#define TAS5716_MISC1_NSHHPEN	BIT(6)
#define TAS5716_MISC1_RPDNEN	BIT(7)

/* miscellaneous register 2 */
#define TAS5716_MISC2_PNDLSL_MASK	0x1c
#define TAS5716_MISC2_PNDLSL_SHIFT	2

#endif /* _ASOC_TAS_5716_H */
