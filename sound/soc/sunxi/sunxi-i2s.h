/*
 * sound\soc\sunxi\i2s\sunxi-i2s.h
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * chenpailin <chenpailin@allwinnertech.com>
 *
 * some simple description for this code
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#ifndef SUNXI_I2S_H_
#define SUNXI_I2S_H_

/*------------------------------------------------------------*/
/* REGISTER definitions */

#define SUNXI_I2S_CTL		0x00 /* Digital Audio Control Register */
#define SUNXI_I2S_FAT0		0x04 /* Digital Audio Format Register 0 */
#define SUNXI_I2S_FAT1		0x08 /* Digital Audio Format Register 1 */
#define SUNXI_I2S_TXFIFO	0x0C /* Digital Audio TX FIFO Register */
#define SUNXI_I2S_RXFIFO	0x10 /* Digital Audio RX FIFO Register */
#define SUNXI_I2S_FCTL		0x14 /* Digital Audio FIFO Control Register */
#define SUNXI_I2S_FSTA		0x18 /* Digital Audio FIFO Status Register */
#define SUNXI_I2S_INT		0x1C /* Digital Audio Interrupt Control Register */
#define SUNXI_I2S_ISTA		0x20 /* Digital Audio Interrupt Status Register */
#define SUNXI_I2S_CLKD		0x24 /* Digital Audio Clock Divide Register */
#define SUNXI_I2S_RXCNT		0x28 /* Digital Audio RX Sample Counter Register */
#define SUNXI_I2S_TXCNT		0x2C /* Digital Audio TX Sample Counter Register */
#define SUNXI_I2S_TXCHSEL	0x30 /* Digital Audio TX Channel Select register */
#define SUNXI_I2S_TXCHMAP	0x34 /* Digital Audio TX Channel Mapping Register */
#define SUNXI_I2S_RXCHSEL	0x38 /* Digital Audio RX Channel Select register */
#define SUNXI_I2S_RXCHMAP	0x3C /* Digital Audio RX Channel Mapping Register */


/* SUNXI_I2S_CTL	0x00 	 Digital Audio Control Register */
#define SUNXI_I2SCTL_SDO3EN		(1<<11)
#define SUNXI_I2SCTL_SDO3EN_MASK	(1<<11)
#define SUNXI_I2SCTL_SDO3EN_SHIFT	11
#define SUNXI_I2SCTL_SDO3EN_WIDTH	1
#define SUNXI_I2SCTL_SDO2EN		(1<<10)
#define SUNXI_I2SCTL_SDO2EN_MASK	(1<<10)
#define SUNXI_I2SCTL_SDO2EN_SHIFT	10
#define SUNXI_I2SCTL_SDO2EN_WIDTH	1
#define SUNXI_I2SCTL_SDO1EN		(1<<9)
#define SUNXI_I2SCTL_SDO1EN_MASK	(1<<9)
#define SUNXI_I2SCTL_SDO1EN_SHIFT	9
#define SUNXI_I2SCTL_SDO1EN_WIDTH	1
#define SUNXI_I2SCTL_SDO0EN		(1<<8)
#define SUNXI_I2SCTL_SDO0EN_MASK	(1<<8)
#define SUNXI_I2SCTL_SDO0EN_SHIFT	8
#define SUNXI_I2SCTL_SDO0EN_WIDTH	1
#define SUNXI_I2SCTL_SDOEN_ALL (SUNXI_I2SCTL_SDO3EN | SUNXI_I2SCTL_SDO2EN | SUNXI_I2SCTL_SDO1EN| SUNXI_I2SCTL_SDO0EN)

#define SUNXI_I2SCTL_ASS		(1<<6)
#define SUNXI_I2SCTL_ASS_MASK		(1<<6)
#define SUNXI_I2SCTL_ASS_SHIFT		6
#define SUNXI_I2SCTL_ASS_WIDTH		1
#define SUNXI_I2SCTL_MS			(1<<5)
#define SUNXI_I2SCTL_MS_MASK		(1<<5)
#define SUNXI_I2SCTL_MS_SHIFT		5
#define SUNXI_I2SCTL_MS_WIDTH		1
#define SUNXI_I2SCTL_PCM		(1<<4)
#define SUNXI_I2SCTL_PCM_MASK		(1<<4)
#define SUNXI_I2SCTL_PCM_SHIFT		4
#define SUNXI_I2SCTL_PCM_WIDTH		1
#define SUNXI_I2SCTL_LOOP		(1<<3)
#define SUNXI_I2SCTL_LOOP_MASK		(1<<3)
#define SUNXI_I2SCTL_LOOP_SHIFT		3
#define SUNXI_I2SCTL_LOOP_WIDTH		1
#define SUNXI_I2SCTL_TXEN		(1<<2)
#define SUNXI_I2SCTL_TXEN_MASK		(1<<2)
#define SUNXI_I2SCTL_TXEN_SHIFT		2
#define SUNXI_I2SCTL_TXEN_WIDTH		1
#define SUNXI_I2SCTL_RXEN		(1<<1)
#define SUNXI_I2SCTL_RXEN_MASK		(1<<1)
#define SUNXI_I2SCTL_RXEN_SHIFT		1
#define SUNXI_I2SCTL_RXEN_WIDTH		1
#define SUNXI_I2SCTL_GEN		(1<<0)
#define SUNXI_I2SCTL_GEN_MASK		(1<<0)
#define SUNXI_I2SCTL_GEN_SHIFT		0
#define SUNXI_I2SCTL_GEN_WIDTH		1

/* SUNXI_I2S_FAT0	0x04 	 Digital Audio Format Register 0 */

#define SUNXI_I2SFAT0_LRCP		(1<<7)
#define SUNXI_I2SFAT0_LRCP_MASK		(1<<7)
#define SUNXI_I2SFAT0_LRCP_SHIFT	7
#define SUNXI_I2SFAT0_LRCP_WIDTH	1
#define SUNXI_I2SFAT0_BCP		(1<<6)
#define SUNXI_I2SFAT0_BCP_MASK		(1<<6)
#define SUNXI_I2SFAT0_BCP_SHIFT		6
#define SUNXI_I2SFAT0_BCP_WIDTH		1
#define SUNXI_I2SFAT0_SR_MASK		(3<<4)
#define SUNXI_I2SFAT0_SR_16BIT		(0<<4)
#define	SUNXI_I2SFAT0_SR_20BIT		(1<<4)
#define SUNXI_I2SFAT0_SR_24BIT		(2<<4)
#define SUNXI_I2SFAT0_SR_SHIFT		4
#define SUNXI_I2SFAT0_SR_WIDTH		2
#define SUNXI_I2SFAT0_WSS_MASK		(3<<2)
#define SUNXI_I2SFAT0_WSS_16BCLK	(0<<2)
#define SUNXI_I2SFAT0_WSS_20BCLK	(1<<2)
#define SUNXI_I2SFAT0_WSS_24BCLK	(2<<2)
#define SUNXI_I2SFAT0_WSS_32BCLK	(3<<2)
#define SUNXI_I2SFAT0_WSS_SHIFT		2
#define SUNXI_I2SFAT0_WSS_WIDTH		2
#define SUNXI_I2SFAT0_FMT_MASK		(3<<0)
#define SUNXI_I2SFAT0_FMT_I2S		(0<<0)
#define SUNXI_I2SFAT0_FMT_LFT		(1<<0)
#define SUNXI_I2SFAT0_FMT_RGT		(2<<0)
#define SUNXI_I2SFAT0_FMT_RVD		(3<<0)
#define SUNXI_I2SFAT0_FMT_SHIFT		0
#define SUNXI_I2SFAT0_FMT_WIDTH		2

/* SUNXI_I2S_FAT1	0x08 	 Digital Audio Format Register 1 */

#define SUNXI_I2SFAT1_SYNCLEN_MASK	(7<<12)
#define SUNXI_I2SFAT1_SYNCLEN_16BCLK	(0<<12)
#define SUNXI_I2SFAT1_SYNCLEN_32BCLK	(1<<12)
#define SUNXI_I2SFAT1_SYNCLEN_64BCLK	(2<<12)
#define SUNXI_I2SFAT1_SYNCLEN_128BCLK	(3<<12)
#define SUNXI_I2SFAT1_SYNCLEN_256BCLK	(4<<12)
#define SUNXI_I2SFAT1_SYNCLEN_SHIFT	12
#define SUNXI_I2SFAT1_SYNCLEN_WIDTH	3
#define SUNXI_I2SFAT1_SYNCOUTEN		(1<<11)
#define SUNXI_I2SFAT1_SYNCOUTEN_MASK	(1<<11)
#define SUNXI_I2SFAT1_SYNCOUTEN_SHIFT	11
#define SUNXI_I2SFAT1_SYNCOUTEN_WIDTH	1
#define SUNXI_I2SFAT1_OUTMUTE		(1<<10)
#define SUNXI_I2SFAT1_OUTMUTE_MASK	(1<<10)
#define SUNXI_I2SFAT1_OUTMUTE_SHIFT	10
#define SUNXI_I2SFAT1_OUTMUTE_WIDTH	1
#define SUNXI_I2SFAT1_MLS		(1<<9)
#define SUNXI_I2SFAT1_MLS_MASK		(1<<9)
#define SUNXI_I2SFAT1_MLS_SHIFT		9
#define SUNXI_I2SFAT1_MLS_WIDTH		1
#define SUNXI_I2SFAT1_SEXT		(1<<8)
#define SUNXI_I2SFAT1_SEXT_MASK		(1<<8)
#define SUNXI_I2SFAT1_SEXT_SHIFT	8
#define SUNXI_I2SFAT1_SEXT_WIDTH	1
#define SUNXI_I2SFAT1_SI_MASK		(3<<6)
#define SUNXI_I2SFAT1_SI_1ST		(0<<6)
#define SUNXI_I2SFAT1_SI_2ND		(1<<6)
#define SUNXI_I2SFAT1_SI_3RD		(2<<6)
#define SUNXI_I2SFAT1_SI_4TH		(3<<6)
#define SUNXI_I2SFAT1_SI_SHIFT		6
#define SUNXI_I2SFAT1_SI_WIDTH		2
#define SUNXI_I2SFAT1_SW		(1<<5)
#define SUNXI_I2SFAT1_SW_MASK		(1<<5)
#define SUNXI_I2SFAT1_SW_SHIFT		5
#define SUNXI_I2SFAT1_SW_WIDTH		1
#define SUNXI_I2SFAT1_SSYNC		(1<<4)
#define SUNXI_I2SFAT1_SSYNC_MASK	(1<<4)
#define SUNXI_I2SFAT1_SSYNC_SHIFT	4
#define SUNXI_I2SFAT1_SSYNC_WIDTH	1
#define SUNXI_I2SFAT1_RXPDM_MASK	(3<<2)
#define SUNXI_I2SFAT1_RXPDM_16PCM	(0<<2)
#define SUNXI_I2SFAT1_RXPDM_8PCM	(1<<2)
#define SUNXI_I2SFAT1_RXPDM_8ULAW	(2<<2)
#define SUNXI_I2SFAT1_RXPDM_8ALAW  	(3<<2)
#define SUNXI_I2SFAT1_RXPDM_SHIFT	2
#define SUNXI_I2SFAT1_RXPDM_WIDTH	2
#define SUNXI_I2SFAT1_TXPDM_MASK	(3<<0)
#define SUNXI_I2SFAT1_TXPDM_16PCM	(0<<0)
#define SUNXI_I2SFAT1_TXPDM_8PCM	(1<<0)
#define SUNXI_I2SFAT1_TXPDM_8ULAW	(2<<0)
#define SUNXI_I2SFAT1_TXPDM_8ALAW  	(3<<0)
#define SUNXI_I2SFAT1_TXPDM_SHIFT	0
#define SUNXI_I2SFAT1_TXPDM_WIDTH	2

/* SUNXI_I2S_TXFIFO	0x0C 	 Digital Audio TX FIFO Register */

/* SUNXI_I2S_RXFIFO	0x10 	 Digital Audio RX FIFO Register */

/* SUNXI_I2S_FCTL	0x14 	 Digital Audio FIFO Control Register */

#define SUNXI_I2SFCTL_FIFOSRC		(1<<31)
#define SUNXI_I2SFCTL_FIFOSRC_MASK	(1<<31)
#define SUNXI_I2SFCTL_FIFOSRC_SHIFT	31
#define SUNXI_I2SFCTL_FIFOSRC_WIDTH	1
#define SUNXI_I2SFCTL_FTX		(1<<25)
#define SUNXI_I2SFCTL_FTX_MASK		(1<<25)
#define SUNXI_I2SFCTL_FTX_SHIFT		25
#define SUNXI_I2SFCTL_FTX_WIDTH		1
#define SUNXI_I2SFCTL_FRX		(1<<24)
#define SUNXI_I2SFCTL_FRX_MASK		(1<<24)
#define SUNXI_I2SFCTL_FRX_SHIFT		24
#define SUNXI_I2SFCTL_FRX_WIDTH		1
#define SUNXI_I2SFCTL_TXTL(x) ((x << SUNXI_I2SFCTL_TXTL_SHIFT) & SUNXI_I2SFCTL_TXTL_MASK)
#define SUNXI_I2SFCTL_TXTL_MASK		(0x3F<<12)
#define SUNXI_I2SFCTL_TXTL_SHIFT	12
#define SUNXI_I2SFCTL_TXTL_WIDTH	7
#define SUNXI_I2SFCTL_RXTL(x) ((x << SUNXI_I2SFCTL_RXTL_SHIFT) & SUNXI_I2SFCTL_RXTL_MASK)
#define SUNXI_I2SFCTL_RXTL_MASK		(0x3F<<4)
#define SUNXI_I2SFCTL_RXTL_SHIFT	4
#define SUNXI_I2SFCTL_RXTL_WIDTH	7
#define SUNXI_I2SFCTL_TXIM_MASK		(1<<2)
#define SUNXI_I2SFCTL_TXIM_MOD0		(0<<2)
#define SUNXI_I2SFCTL_TXIM_MOD1		(1<<2)
#define SUNXI_I2SFCTL_TXIM_SHIFT	2
#define SUNXI_I2SFCTL_TXIM_WIDTH	1
#define SUNXI_I2SFCTL_RXOM_MASK		(3<<0)
#define SUNXI_I2SFCTL_RXOM_MOD0		(0<<0)
#define SUNXI_I2SFCTL_RXOM_MOD1		(1<<0)
#define SUNXI_I2SFCTL_RXOM_MOD2		(2<<0)
#define SUNXI_I2SFCTL_RXOM_MOD3		(3<<0)
#define SUNXI_I2SFCTL_RXOM_SHIFT	0
#define SUNXI_I2SFCTL_RXOM_WIDTH	2

/* SUNXI_I2S_FSTA	0x18 	 Digital Audio FIFO Status Register */

#define SUNXI_I2SFSTA_TXE		(1<<28)
#define SUNXI_I2SFSTA_TXE_MASK		(1<<28)
#define SUNXI_I2SFSTA_TXE_SHIFT		28
#define SUNXI_I2SFSTA_TXE_WIDTH		1
#define SUNXI_I2SFSTA_TXECNT_MASK	(0xFF<<16)
#define SUNXI_I2SFSTA_TXECNT_SHIFT	16
#define SUNXI_I2SFSTA_TXECNT_WIDTH	8
#define SUNXI_I2SFSTA_RXA		(1<<8)
#define SUNXI_I2SFSTA_RXA_MASK		(1<<8)
#define SUNXI_I2SFSTA_RXA_SHIFT		8
#define SUNXI_I2SFSTA_RXA_WIDTH		1
#define SUNXI_I2SFSTA_RXACNT_MASK	(0x3F<<0)
#define SUNXI_I2SFSTA_RXACNT_SHIFT	0
#define SUNXI_I2SFSTA_RXACNT_WIDTH	7

/* SUNXI_I2S_INT	0x1C 	 Digital Audio Interrupt Control Register */

#define SUNXI_I2SINT_TXDRQEN		(1<<7)
#define SUNXI_I2SINT_TXDRQEN_MASK	(1<<7)
#define SUNXI_I2SINT_TXDRQEN_SHIFT	7
#define SUNXI_I2SINT_TXDRQEN_WIDTH	1
#define SUNXI_I2SINT_TXUIEN		(1<<6)
#define SUNXI_I2SINT_TXUIEN_MASK	(1<<6)
#define SUNXI_I2SINT_TXUIEN_SHIFT	6
#define SUNXI_I2SINT_TXUIEN_WIDTH	1
#define SUNXI_I2SINT_TXOIEN		(1<<5)
#define SUNXI_I2SINT_TXOIEN_MASK	(1<<5)
#define SUNXI_I2SINT_TXOIEN_SHIFT	5
#define SUNXI_I2SINT_TXOIEN_WIDTH	1
#define SUNXI_I2SINT_TXEIEN		(1<<4)
#define SUNXI_I2SINT_TXEIEN_MASK	(1<<4)
#define SUNXI_I2SINT_TXEIEN_SHIFT	4
#define SUNXI_I2SINT_TXEIEN_WIDTH	1
#define SUNXI_I2SINT_RXDRQEN		(1<<3)
#define SUNXI_I2SINT_RXDRQEN_MASK	(1<<3)
#define SUNXI_I2SINT_RXDRQEN_SHIFT	3
#define SUNXI_I2SINT_RXDRQEN_WIDTH	1
#define SUNXI_I2SINT_RXUIEN		(1<<2)
#define SUNXI_I2SINT_RXUIEN_MASK	(1<<2)
#define SUNXI_I2SINT_RXUIEN_SHIFT	2
#define SUNXI_I2SINT_RXUIEN_WIDTH	1
#define SUNXI_I2SINT_RXOIEN		(1<<1)
#define SUNXI_I2SINT_RXOIEN_MASK	(1<<1)
#define SUNXI_I2SINT_RXOIEN_SHIFT	1
#define SUNXI_I2SINT_RXOIEN_WIDTH	1
#define SUNXI_I2SINT_RXAIEN		(1<<0)
#define SUNXI_I2SINT_RXAIEN_MASK	(1<<0)
#define SUNXI_I2SINT_RXAIEN_SHIFT	0
#define SUNXI_I2SINT_RXAIEN_WIDTH	1

/* SUNXI_I2S_ISTA	0x20 	 Digital Audio Interrupt Status Register */

#define SUNXI_I2SSTA_TXUIEN		(1<<6)
#define SUNXI_I2SSTA_TXUIEN_MASK	(1<<6)
#define SUNXI_I2SSTA_TXUIEN_SHIFT	6
#define SUNXI_I2SSTA_TXUIEN_WIDTH	1
#define SUNXI_I2SSTA_TXOIEN		(1<<5)
#define SUNXI_I2SSTA_TXOIEN_MASK	(1<<5)
#define SUNXI_I2SSTA_TXOIEN_SHIFT	5
#define SUNXI_I2SSTA_TXOIEN_WIDTH	1
#define SUNXI_I2SSTA_TXEIEN		(1<<4)
#define SUNXI_I2SSTA_TXEIEN_MASK	(1<<4)
#define SUNXI_I2SSTA_TXEIEN_SHIFT	4
#define SUNXI_I2SSTA_TXEIEN_WIDTH	1
#define SUNXI_I2SSTA_RXUIEN		(1<<2)
#define SUNXI_I2SSTA_RXUIEN_MASK	(1<<2)
#define SUNXI_I2SSTA_RXUIEN_SHIFT	2
#define SUNXI_I2SSTA_RXUIEN_WIDTH	1
#define SUNXI_I2SSTA_RXOIEN		(1<<1)
#define SUNXI_I2SSTA_RXOIEN_MASK	(1<<1)
#define SUNXI_I2SSTA_RXOIEN_SHIFT	1
#define SUNXI_I2SSTA_RXOIEN_WIDTH	1
#define SUNXI_I2SSTA_RXAIEN		(1<<0)
#define SUNXI_I2SSTA_RXAIEN_MASK	(1<<0)
#define SUNXI_I2SSTA_RXAIEN_SHIFT	0
#define SUNXI_I2SSTA_RXAIEN_WIDTH	1

/* SUNXI_I2S_CLKD	0x24 	 Digital Audio Clock Divide Register */

#define SUNXI_I2SCLKD_MCLKOEN		(1<<7)
#define SUNXI_I2SCLKD_MCLKOEN_MASK	(1<<7)
#define SUNXI_I2SCLKD_MCLKOEN_SHIFT	7
#define SUNXI_I2SCLKD_MCLKOEN_WIDTH	1
#define SUNXI_I2SCLKD_BCLKDIV_MASK	(7<<4)
#define SUNXI_I2SCLKD_BCLKDIV_2		(0<<4)
#define SUNXI_I2SCLKD_BCLKDIV_4		(1<<4)
#define SUNXI_I2SCLKD_BCLKDIV_6		(2<<4)
#define SUNXI_I2SCLKD_BCLKDIV_8		(3<<4)
#define SUNXI_I2SCLKD_BCLKDIV_12	(4<<4)
#define SUNXI_I2SCLKD_BCLKDIV_16	(5<<4)
#define SUNXI_I2SCLKD_BCLKDIV_32	(6<<4)
#define SUNXI_I2SCLKD_BCLKDIV_64	(7<<4)
#define SUNXI_I2SCLKD_BCLKDIV_SHIFT	4
#define SUNXI_I2SCLKD_BCLKDIV_WIDTH	3
#define SUNXI_I2SCLKD_MCLKDIV_MASK	(0xF<<0)
#define SUNXI_I2SCLKD_MCLKDIV_1		(0<<0)
#define SUNXI_I2SCLKD_MCLKDIV_2		(1<<0)
#define SUNXI_I2SCLKD_MCLKDIV_4		(2<<0)
#define SUNXI_I2SCLKD_MCLKDIV_6		(3<<0)
#define SUNXI_I2SCLKD_MCLKDIV_8		(4<<0)
#define SUNXI_I2SCLKD_MCLKDIV_12	(5<<0)
#define SUNXI_I2SCLKD_MCLKDIV_16	(6<<0)
#define SUNXI_I2SCLKD_MCLKDIV_24	(7<<0)
#define SUNXI_I2SCLKD_MCLKDIV_32	(8<<0)
#define SUNXI_I2SCLKD_MCLKDIV_48	(9<<0)
#define SUNXI_I2SCLKD_MCLKDIV_64	(10<<0)
#define SUNXI_I2SCLKD_MCLKDIV_SHIFT	0
#define SUNXI_I2SCLKD_MCLKDIV_WIDTH	4

/* SUNXI_I2S_RXCNT	0x28 	 Digital Audio RX Sample Counter Register */

/* SUNXI_I2S_TXCNT	0x2C 	 Digital Audio TX Sample Counter Register */

/* SUNXI_I2S_TXCHSEL	0x30 	 Digital Audio TX Channel Select register */

#define SUNXI_I2STXCHSEL_CHNUM_MASK	(7<<0)
#define SUNXI_I2STXCHSEL_CHNUM_SHIFT	0
#define SUNXI_I2STXCHSEL_CHNUM_WIDTH	3

/* SUNXI_I2S_TXCHMAP	0x34 	 Digital Audio TX Channel Mapping Register */

#define SUNXI_I2STXCHMAP_CH7_MASK	(7<<28)
#define SUNXI_I2STXCHMAP_CH7_SHIFT	28
#define SUNXI_I2STXCHMAP_CH7_WIDTH	3
#define SUNXI_I2STXCHMAP_CH6_MASK	(7<<24)
#define SUNXI_I2STXCHMAP_CH6_SHIFT	24
#define SUNXI_I2STXCHMAP_CH6_WIDTH	3
#define SUNXI_I2STXCHMAP_CH5_MASK	(7<<20)
#define SUNXI_I2STXCHMAP_CH5_SHIFT	20
#define SUNXI_I2STXCHMAP_CH5_WIDTH	3
#define SUNXI_I2STXCHMAP_CH4_MASK	(7<<16)
#define SUNXI_I2STXCHMAP_CH4_SHIFT	16
#define SUNXI_I2STXCHMAP_CH4_WIDTH	3
#define SUNXI_I2STXCHMAP_CH3_MASK	(7<<12)
#define SUNXI_I2STXCHMAP_CH3_SHIFT	12
#define SUNXI_I2STXCHMAP_CH3_WIDTH	3
#define SUNXI_I2STXCHMAP_CH2_MASK	(7<<8)
#define SUNXI_I2STXCHMAP_CH2_SHIFT	8
#define SUNXI_I2STXCHMAP_CH2_WIDTH	3
#define SUNXI_I2STXCHMAP_CH1_MASK	(7<<4)
#define SUNXI_I2STXCHMAP_CH1_SHIFT	4
#define SUNXI_I2STXCHMAP_CH1_WIDTH	3
#define SUNXI_I2STXCHMAP_CH0_MASK	(7<<0)
#define SUNXI_I2STXCHMAP_CH0_SHIFT	0
#define SUNXI_I2STXCHMAP_CH0_WIDTH	3

/* SUNXI_I2S_RXCHSEL	0x38 	 Digital Audio RX Channel Select register */

#define SUNXI_I2SRXCHSEL_CHNUM_MASK	(7<<0)
#define SUNXI_I2SRXCHSEL_CHNUM_SHIFT	0
#define SUNXI_I2SRXCHSEL_CHNUM_WIDTH	3

/* SUNXI_I2S_RXCHMAP	0x3C 	 Digital Audio RX Channel Mapping Register */

#define SUNXI_I2SRXCHMAP_CH3_MASK	(7<<12)
#define SUNXI_I2SRXCHMAP_CH3_SHIFT	12
#define SUNXI_I2SRXCHMAP_CH3_WIDTH	3
#define SUNXI_I2SRXCHMAP_CH2_MASK	(7<<8)
#define SUNXI_I2SRXCHMAP_CH2_SHIFT	8
#define SUNXI_I2SRXCHMAP_CH2_WIDTH	3
#define SUNXI_I2SRXCHMAP_CH1_MASK	(7<<4)
#define SUNXI_I2SRXCHMAP_CH1_SHIFT	4
#define SUNXI_I2SRXCHMAP_CH1_WIDTH	3
#define SUNXI_I2SRXCHMAP_CH0_MASK	(7<<0)
#define SUNXI_I2SRXCHMAP_CH0_SHIFT	0
#define SUNXI_I2SRXCHMAP_CH0_WIDTH	3


/*------------------------------------------------------------*/
/* Clock dividers */
#define SUNXI_DIV_MCLK	0
#define SUNXI_DIV_BCLK	1

#define SUNXI_I2SCLKD_MCLK_MASK   0x0f
#define SUNXI_I2SCLKD_MCLK_OFFS   0
#define SUNXI_I2SCLKD_BCLK_MASK   0x070
#define SUNXI_I2SCLKD_BCLK_OFFS   4
#define SUNXI_I2SCLKD_MCLKEN_OFFS 7

/* Supported SoC families - used for quirks */
enum sunxi_soc_family {
	SUN4IA,	/* A10 SoC - revision A */
	SUN4I,	/* A10 SoC - later revisions */
	SUN5I,	/* A10S/A13 SoCs */
	SUN7I,	/* A20 SoC */
};

struct sunxi_priv {
	struct regmap *regmap;
	struct clk *clk_apb, *clk_pll2, *clk_module;

	enum sunxi_soc_family revision;

	struct snd_dmaengine_dai_dma_data playback_dma_data;
	struct snd_dmaengine_dai_dma_data capture_dma_data;

	u32 slave;		//0: master, 1: slave
	u32 mono;		//0: stereo, 1: mono
	u32 samp_fs;		//audio sample rate (unit in kHz)
	u32 samp_res;		//16 bits, 20 bits , 24 bits, 32 bits)
	u32 samp_format;	//audio sample format (0: standard I2S, 1: left-justified, 2: right-justified, 3: pcm)
	u32 ws_size;		//16 BCLK, 20 BCLK, 24 BCLK, 32 BCLK)
	u32 mclk_rate;		//mclk frequency divide by fs (128fs, 192fs, 256fs, 384fs, 512fs, 768fs)
	u32 lrc_pol;		//LRC clock polarity (0: normal ,1: inverted)
	u32 bclk_pol;		//BCLK polarity (0: normal, 1: inverted)
	u32 pcm_txtype;		//PCM transmitter type (0: 16-bits linear mode, 1: 8-bits linear mode, 2: u-law, 3: A-law)
	u32 pcm_rxtype;		//PCM receiver type  (0: 16-bits linear mode, 1: 8-bits linear mode, 2: u-law, 3: A-law)
	u32 pcm_sw;		//PCM slot width (8: 8 bits, 16: 16 bits)
	u32 pcm_sync_period;	//PCM sync period (16/32/64/128/256)
	u32 pcm_sync_type;	//PCM sync symbol size (0: short sync, 1: long sync)
	u32 pcm_start_slot;	//PCM start slot index (1--4)
	u32 pcm_lsb_first;	//0: MSB first, 1: LSB first
	u32 pcm_ch_num;		//PCM channel number (1: one channel, 2: two channel)
};

#endif
