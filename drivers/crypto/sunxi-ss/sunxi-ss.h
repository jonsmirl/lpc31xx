/*
 * sunxi-ss.c - hardware cryptographic accelerator for Allwinner A20 SoC
 *
 * Copyright (C) 2013-2014 Corentin LABBE <clabbe.montjoie@gmail.com>
 *
 * Support AES cipher with 128,192,256 bits keysize.
 * Support MD5 and SHA1 hash algorithms.
 * Support DES and 3DES
 * Support PRNG
 *
 * You could find the datasheet at
 * http://dl.linux-sunxi.org/A20/A20%20User%20Manual%202013-03-22.pdf
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2 of the License
 */

#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <crypto/scatterwalk.h>
#include <linux/scatterlist.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <crypto/md5.h>
#include <crypto/sha.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <crypto/aes.h>
#include <crypto/des.h>

#define SS_CTL            0x00
#define SS_KEY0           0x04
#define SS_KEY1           0x08
#define SS_KEY2           0x0C
#define SS_KEY3           0x10
#define SS_KEY4           0x14
#define SS_KEY5           0x18
#define SS_KEY6           0x1C
#define SS_KEY7           0x20

#define SS_IV0            0x24
#define SS_IV1            0x28
#define SS_IV2            0x2C
#define SS_IV3            0x30

#define SS_CNT0           0x34
#define SS_CNT1           0x38
#define SS_CNT2           0x3C
#define SS_CNT3           0x40

#define SS_FCSR           0x44
#define SS_ICSR           0x48

#define SS_MD0            0x4C
#define SS_MD1            0x50
#define SS_MD2            0x54
#define SS_MD3            0x58
#define SS_MD4            0x5C

#define SS_RXFIFO         0x200
#define SS_TXFIFO         0x204

/* SS_CTL configuration values */

/* PRNG generator mode - bit 15 */
#define SS_PRNG_ONESHOT		(0 << 15)
#define SS_PRNG_CONTINUE	(1 << 15)

/* SS operation mode - bits 12-13 */
#define SS_ECB			(0 << 12)
#define SS_CBC			(1 << 12)
#define SS_CNT			(2 << 12)

/* Counter width for CNT mode - bits 10-11 */
#define SS_CNT_16BITS		(0 << 10)
#define SS_CNT_32BITS		(1 << 10)
#define SS_CNT_64BITS		(2 << 10)

/* Key size for AES - bits 8-9 */
#define SS_AES_128BITS		(0 << 8)
#define SS_AES_192BITS		(1 << 8)
#define SS_AES_256BITS		(2 << 8)

/* Operation direction - bit 7 */
#define SS_ENCRYPTION		(0 << 7)
#define SS_DECRYPTION		(1 << 7)

/* SS Method - bits 4-6 */
#define SS_OP_AES		(0 << 4)
#define SS_OP_DES		(1 << 4)
#define SS_OP_3DES		(2 << 4)
#define SS_OP_SHA1		(3 << 4)
#define SS_OP_MD5		(4 << 4)
#define SS_OP_PRNG		(5 << 4)

/* Data end bit - bit 2 */
#define SS_DATA_END		(1 << 2)

/* PRNG start bit - bit 1 */
#define SS_PRNG_START		(1 << 1)

/* SS Enable bit - bit 0 */
#define SS_DISABLED		(0 << 0)
#define SS_ENABLED		(1 << 0)

/* SS_FCSR configuration values */
/* RX FIFO status - bit 30 */
#define SS_RXFIFO_FREE		(1 << 30)

/* RX FIFO empty spaces - bits 24-29 */
#define SS_RXFIFO_SPACES(val)	(((val) >> 24) & 0x3f)

/* TX FIFO status - bit 22 */
#define SS_TXFIFO_AVAILABLE	(1 << 22)

/* TX FIFO available spaces - bits 16-21 */
#define SS_TXFIFO_SPACES(val)	(((val) >> 16) & 0x3f)

#define SS_RXFIFO_EMP_INT_PENDING	(1 << 10)
#define SS_TXFIFO_AVA_INT_PENDING	(1 << 8)
#define SS_RXFIFO_EMP_INT_ENABLE	(1 << 2)
#define SS_TXFIFO_AVA_INT_ENABLE	(1 << 0)

/* SS_ICSR configuration values */
#define SS_ICS_DRQ_ENABLE		(1 << 4)

struct sunxi_ss_ctx {
	void __iomem *base;
	int irq;
	struct clk *busclk;
	struct clk *ssclk;
	struct device *dev;
	struct resource *res;
	void *buf_in; /* pointer to data to be uploaded to the device */
	size_t buf_in_size; /* size of buf_in */
	void *buf_out;
	size_t buf_out_size;
	struct mutex lock; /* control the use of the device */
	struct mutex bufout_lock; /* control the use of buf_out*/
	struct mutex bufin_lock; /* control the sue of buf_in*/
};

struct sunxi_req_ctx {
	u8 key[AES_MAX_KEY_SIZE * 8];
	u32 keylen;
	u32 mode;
	u64 byte_count; /* number of bytes "uploaded" to the device */
	u32 waitbuf; /* a partial word waiting to be completed and
			uploaded to the device */
	/* number of bytes to be uploaded in the waitbuf word */
	unsigned int nbwait;
};

int sunxi_ss_aes_init(void);

