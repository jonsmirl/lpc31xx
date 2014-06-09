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
#include "sunxi-ss-cipher.h"

int sunxi_cipher_init(struct crypto_tfm *tfm)
{
	struct sunxi_req_ctx *op = crypto_tfm_ctx(tfm);
	memset(op, 0, sizeof(struct sunxi_req_ctx));
	return 0;
}

void sunxi_cipher_exit(struct crypto_tfm *tfm)
{
}

int sunxi_aes_poll(struct ablkcipher_request *areq)
{
	u32 tmp;
	struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(areq);
	struct sunxi_req_ctx *op = crypto_ablkcipher_ctx(tfm);
	unsigned int ivsize = crypto_ablkcipher_ivsize(tfm);
	/* when activating SS, the default FIFO space is 32 */
	u32 rx_cnt = 32;
	u32 tx_cnt = 0;
	u32 v;
	int i;
	struct scatterlist *in_sg;
	struct scatterlist *out_sg;
	void *src_addr;
	void *dst_addr;
	unsigned int ileft = areq->nbytes;
	unsigned int oleft = areq->nbytes;
	unsigned int sgileft = areq->src->length;
	unsigned int sgoleft = areq->dst->length;
	unsigned int todo;
	u32 *src32;
	u32 *dst32;

	tmp = op->mode;
	tmp |= SS_ENABLED;

	in_sg = areq->src;
	out_sg = areq->dst;
	if (areq->src == NULL || areq->dst == NULL) {
		dev_err(ss->dev, "ERROR: Some SGs are NULL %u\n", areq->nbytes);
		return -1;
	}
	mutex_lock(&ss->lock);
	if (areq->info != NULL) {
		for (i = 0; i < op->keylen; i += 4) {
			v = *(u32 *)(op->key + i);
			writel(v, ss->base + SS_KEY0 + i);
		}
		for (i = 0; i < 4 && i < ivsize / 4; i++) {
			v = *(u32 *)(areq->info + i * 4);
			writel(v, ss->base + SS_IV0 + i * 4);
		}
	}
	writel(tmp, ss->base + SS_CTL);

	/* If we have only one SG, we can use kmap_atomic */
	if (sg_next(in_sg) == NULL && sg_next(out_sg) == NULL) {
		src_addr = kmap_atomic(sg_page(in_sg)) + in_sg->offset;
		if (src_addr == NULL) {
			dev_err(ss->dev, "kmap_atomic error for src SG\n");
			writel(0, ss->base + SS_CTL);
			mutex_unlock(&ss->lock);
			return -1;
		}
		dst_addr = kmap_atomic(sg_page(out_sg)) + out_sg->offset;
		if (dst_addr == NULL) {
			dev_err(ss->dev, "kmap_atomic error for dst SG\n");
			writel(0, ss->base + SS_CTL);
			mutex_unlock(&ss->lock);
			kunmap_atomic(src_addr);
			return -1;
		}
		src32 = (u32 *)src_addr;
		dst32 = (u32 *)dst_addr;
		ileft = areq->nbytes / 4;
		oleft = areq->nbytes / 4;
		do {
			if (ileft > 0 && rx_cnt > 0) {
				todo = min(rx_cnt, ileft);
				ileft -= todo;
				do {
					writel_relaxed(*src32++,
						       ss->base +
						       SS_RXFIFO);
					todo--;
				} while (todo > 0);
			}
			if (tx_cnt > 0) {
				todo = min(tx_cnt, oleft);
				oleft -= todo;
				do {
					*dst32++ = readl_relaxed(ss->base +
								SS_TXFIFO);
					todo--;
				} while (todo > 0);
			}
			tmp = readl_relaxed(ss->base + SS_FCSR);
			rx_cnt = SS_RXFIFO_SPACES(tmp);
			tx_cnt = SS_TXFIFO_SPACES(tmp);
		} while (oleft > 0);
		writel(0, ss->base + SS_CTL);
		mutex_unlock(&ss->lock);
		kunmap_atomic(src_addr);
		kunmap_atomic(dst_addr);
		return 0;
	}

	/* If we have more than one SG, we cannot use kmap_atomic since
	 * we hold the mapping too long*/
	src_addr = kmap(sg_page(in_sg)) + in_sg->offset;
	if (src_addr == NULL) {
		dev_err(ss->dev, "KMAP error for src SG\n");
		return -1;
	}
	dst_addr = kmap(sg_page(out_sg)) + out_sg->offset;
	if (dst_addr == NULL) {
		kunmap(sg_page(in_sg));
		dev_err(ss->dev, "KMAP error for dst SG\n");
		return -1;
	}
	src32 = (u32 *)src_addr;
	dst32 = (u32 *)dst_addr;
	ileft = areq->nbytes / 4;
	oleft = areq->nbytes / 4;
	sgileft = in_sg->length / 4;
	sgoleft = out_sg->length / 4;
	do {
		tmp = readl_relaxed(ss->base + SS_FCSR);
		rx_cnt = SS_RXFIFO_SPACES(tmp);
		tx_cnt = SS_TXFIFO_SPACES(tmp);
		todo = min3(rx_cnt, ileft, sgileft);
		if (todo > 0) {
			ileft -= todo;
			sgileft -= todo;
		}
		while (todo > 0) {
			writel_relaxed(*src32++, ss->base + SS_RXFIFO);
			todo--;
		}
		if (in_sg != NULL && sgileft == 0 && ileft > 0) {
			kunmap(sg_page(in_sg));
			in_sg = sg_next(in_sg);
			if (in_sg != NULL && in_sg->length == 0)
				in_sg = sg_next(in_sg);
			if (in_sg != NULL && in_sg->length == 0)
				in_sg = sg_next(in_sg);
			if (in_sg != NULL && ileft > 0) {
				src_addr = kmap(sg_page(in_sg)) + in_sg->offset;
				if (src_addr == NULL) {
					dev_err(ss->dev, "KMAP error for src SG\n");
					return -1;
				}
				src32 = src_addr;
				sgileft = in_sg->length / 4;
			}
		}
		/* do not test oleft since when oleft == 0 we have finished */
		todo = min3(tx_cnt, oleft, sgoleft);
		if (todo > 0) {
			oleft -= todo;
			sgoleft -= todo;
		}
		while (todo > 0) {
			*dst32++ = readl_relaxed(ss->base + SS_TXFIFO);
			todo--;
		}
		if (out_sg != NULL && sgoleft == 0 && oleft >= 0) {
			kunmap(sg_page(out_sg));
			out_sg = sg_next(out_sg);
			if (out_sg != NULL && out_sg->length == 0)
				out_sg = sg_next(out_sg);
			if (out_sg != NULL && oleft > 0) {
				dst_addr = kmap(sg_page(out_sg)) +
					out_sg->offset;
				if (dst_addr == NULL) {
					dev_err(ss->dev, "KMAP error\n");
					return -1;
				}
				dst32 = dst_addr;
				sgoleft = out_sg->length / 4;
			}
		}
	} while (oleft > 0);

	writel(0, ss->base + SS_CTL);
	mutex_unlock(&ss->lock);
	return 0;
}

/* Pure CPU way of doing DES/3DES with SS
 * Since DES and 3DES SGs could be smaller than 4 bytes, I use sg_copy_to_buffer
 * for "linearize" them.
 * The problem with that is that I alloc (2 x areq->nbytes) for buf_in/buf_out
 * TODO: change this system
 * SGsrc -> buf_in -> SS -> buf_out -> SGdst */
int sunxi_des_poll(struct ablkcipher_request *areq)
{
	u32 tmp, value;
	size_t nb_in_sg_tx, nb_in_sg_rx;
	size_t ir, it;
	struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(areq);
	struct sunxi_req_ctx *op = crypto_ablkcipher_ctx(tfm);
	unsigned int ivsize = crypto_ablkcipher_ivsize(tfm);
	u32 tx_cnt = 0;
	u32 rx_cnt = 0;
	u32 v;
	int i;
	int no_chunk = 1;

	/* if we have only SGs with size multiple of 4,
	 * we can use the SS AES function */
	struct scatterlist *in_sg;
	struct scatterlist *out_sg;
	in_sg = areq->src;
	out_sg = areq->dst;

	while (in_sg != NULL && no_chunk == 1) {
		if ((in_sg->length % 4) != 0)
			no_chunk = 0;
		in_sg = sg_next(in_sg);
	}
	while (out_sg != NULL && no_chunk == 1) {
		if ((out_sg->length % 4) != 0)
			no_chunk = 0;
		out_sg = sg_next(out_sg);
	}

	if (no_chunk == 1)
		return sunxi_aes_poll(areq);
	in_sg = areq->src;
	out_sg = areq->dst;

	tmp = op->mode;
	tmp |= SS_ENABLED;

	nb_in_sg_rx = sg_nents(in_sg);
	nb_in_sg_tx = sg_nents(out_sg);

	mutex_lock(&ss->bufin_lock);
	if (ss->buf_in == NULL) {
		ss->buf_in = kmalloc(areq->nbytes, GFP_KERNEL);
		ss->buf_in_size = areq->nbytes;
	} else {
		if (areq->nbytes > ss->buf_in_size) {
			kfree(ss->buf_in);
			ss->buf_in = kmalloc(areq->nbytes, GFP_KERNEL);
			ss->buf_in_size = areq->nbytes;
		}
	}
	if (ss->buf_in == NULL) {
		ss->buf_in_size = 0;
		mutex_unlock(&ss->bufin_lock);
		dev_err(ss->dev, "Unable to allocate pages.\n");
		return -ENOMEM;
	}
	if (ss->buf_out == NULL) {
		mutex_lock(&ss->bufout_lock);
		ss->buf_out = kmalloc(areq->nbytes, GFP_KERNEL);
		if (ss->buf_out == NULL) {
			ss->buf_out_size = 0;
			mutex_unlock(&ss->bufout_lock);
			dev_err(ss->dev, "Unable to allocate pages.\n");
			return -ENOMEM;
		}
		ss->buf_out_size = areq->nbytes;
		mutex_unlock(&ss->bufout_lock);
	} else {
		if (areq->nbytes > ss->buf_out_size) {
			mutex_lock(&ss->bufout_lock);
			kfree(ss->buf_out);
			ss->buf_out = kmalloc(areq->nbytes, GFP_KERNEL);
			if (ss->buf_out == NULL) {
				ss->buf_out_size = 0;
				mutex_unlock(&ss->bufout_lock);
				dev_err(ss->dev, "Unable to allocate pages.\n");
				return -ENOMEM;
			}
			ss->buf_out_size = areq->nbytes;
			mutex_unlock(&ss->bufout_lock);
		}
	}

	sg_copy_to_buffer(areq->src, nb_in_sg_rx, ss->buf_in, areq->nbytes);

	ir = 0;
	it = 0;
	mutex_lock(&ss->lock);
	if (areq->info != NULL) {
		for (i = 0; i < op->keylen; i += 4) {
			v = *(u32 *)(op->key + i);
			writel(v, ss->base + SS_KEY0 + i);
		}
		for (i = 0; i < 4 && i < ivsize / 4; i++) {
			v = *(u32 *)(areq->info + i * 4);
			writel(v, ss->base + SS_IV0 + i * 4);
		}
	}
	writel(tmp, ss->base + SS_CTL);

	do {
		if (rx_cnt == 0 || tx_cnt == 0) {
			tmp = readl(ss->base + SS_FCSR);
			rx_cnt = SS_RXFIFO_SPACES(tmp);
			tx_cnt = SS_TXFIFO_SPACES(tmp);
		}
		if (rx_cnt > 0 && ir < areq->nbytes) {
			do {
				value = *(u32 *)(ss->buf_in + ir);
				writel(value, ss->base + SS_RXFIFO);
				ir += 4;
				rx_cnt--;
			} while (rx_cnt > 0 && ir < areq->nbytes);
		}
		if (tx_cnt > 0 && it < areq->nbytes) {
			do {
				if (ir <= it)
					dev_warn(ss->dev, "ANORMAL %u %u\n",
							ir, it);
				value = readl(ss->base + SS_TXFIFO);
				*(u32 *)(ss->buf_out + it) = value;
				it += 4;
				tx_cnt--;
			} while (tx_cnt > 0 && it < areq->nbytes);
		}
		if (ir == areq->nbytes) {
			mutex_unlock(&ss->bufin_lock);
			ir++;
		}
	} while (it < areq->nbytes);

	writel(0, ss->base + SS_CTL);
	mutex_unlock(&ss->lock);

	/* a simple optimization, since we dont need the hardware for this copy
	 * we release the lock and do the copy. With that we gain 5/10% perf */
	mutex_lock(&ss->bufout_lock);
	sg_copy_from_buffer(areq->dst, nb_in_sg_tx, ss->buf_out, areq->nbytes);

	mutex_unlock(&ss->bufout_lock);
	return 0;
}
