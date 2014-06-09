#include "sunxi-ss-hash.h"

/* sunxi_hash_init: initialize request context
 * Activate the SS, and configure it for MD5 or SHA1
 */
int sunxi_hash_init(struct ahash_request *areq)
{
	const char *hash_type;
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(areq);
	struct sunxi_req_ctx *op = crypto_ahash_ctx(tfm);
	u32 tmp = SS_ENABLED;

	mutex_lock(&ss->lock);

	hash_type = crypto_tfm_alg_name(areq->base.tfm);

	op->byte_count = 0;
	op->nbwait = 0;
	op->waitbuf = 0;

	/* Enable and configure SS for MD5 or SHA1 */
	if (strcmp(hash_type, "sha1") == 0) {
		tmp |= SS_OP_SHA1;
		op->mode = SS_OP_SHA1;
	} else {
		tmp |= SS_OP_MD5;
		op->mode = SS_OP_MD5;
	}

	writel(tmp, ss->base + SS_CTL);
	return 0;
}

/*
 * sunxi_hash_update: update hash engine
 *
 * Could be used for both SHA1 and MD5
 * Write data by step of 32bits and put then in the SS.
 * The remaining data is stored (nbwait bytes) in op->waitbuf
 * As an optimisation, we do not check RXFIFO_SPACES, since SS handle
 * the FIFO faster than our writes
 */
int sunxi_hash_update(struct ahash_request *areq)
{
	u32 v;
	unsigned int i = 0;/* bytes read, to be compared to areq->nbytes */
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(areq);
	struct sunxi_req_ctx *op = crypto_ahash_ctx(tfm);
	struct scatterlist *in_sg;
	unsigned int in_i = 0;/* advancement in the current SG */
	void *src_addr;

	u8 *waitbuf = (u8 *)(&op->waitbuf);

	if (areq->nbytes == 0)
		return 0;

	in_sg = areq->src;
	do {
		src_addr = kmap(sg_page(in_sg)) + in_sg->offset;
		/* step 1, if some bytes remains from last SG,
		 * try to complete them to 4 and sent its */
		if (op->nbwait > 0) {
			while (op->nbwait < 4 && i < areq->nbytes &&
					in_i < in_sg->length) {
				waitbuf[op->nbwait] = *(u8 *)(src_addr + in_i);
				i++;
				in_i++;
				op->nbwait++;
			}
			if (op->nbwait == 4) {
				writel(op->waitbuf, ss->base + SS_RXFIFO);
				op->byte_count += 4;
				op->nbwait = 0;
				op->waitbuf = 0;
			}
		}
		/* step 2, main loop, read data 4bytes at a time */
		while (i < areq->nbytes && areq->nbytes - i >= 4 &&
				in_i < in_sg->length &&
				in_sg->length - in_i >= 4) {
			v = *(u32 *)(src_addr + in_i);
			writel_relaxed(v, ss->base + SS_RXFIFO);
			i += 4;
			op->byte_count += 4;
			in_i += 4;
		}
		/* step 3, if we have less than 4 bytes, copy them in waitbuf
		 * no need to check for op->nbwait < 4 since we cannot have
		 * more than 4 bytes remaining */
		if (in_i < in_sg->length && in_sg->length - in_i < 4 &&
				i < areq->nbytes) {
			do {
				waitbuf[op->nbwait] = *(u8 *)(src_addr + in_i);
				op->nbwait++;
				in_i++;
				i++;
			} while (in_i < in_sg->length && i < areq->nbytes);
		}
		/* we have finished the current SG, try next one */
		kunmap(sg_page(in_sg));
		in_sg = sg_next(in_sg);
		in_i = 0;
	} while (in_sg != NULL && i < areq->nbytes);
	return 0;
}

/*
 * sunxi_hash_final: finalize hashing operation
 *
 * If we have some remaining bytes, send it.
 * Then ask the SS for finalizing the hash
 */
int sunxi_hash_final(struct ahash_request *areq)
{
	u32 v;
	unsigned int i;
	int zeros;
	unsigned int index, padlen;
	__be64 bits;
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(areq);
	struct sunxi_req_ctx *op = crypto_ahash_ctx(tfm);

	if (op->nbwait > 0) {
		op->waitbuf |= ((1 << 7) << (op->nbwait * 8));
		writel(op->waitbuf, ss->base + SS_RXFIFO);
	} else {
		writel((1 << 7), ss->base + SS_RXFIFO);
	}

	/* number of space to pad to obtain 64o minus 8(size) minus 4 (final 1)
	 * example len=0
	 * example len=56
	 * */

	/* we have already send 4 more byte of which nbwait data */
	if (op->mode == SS_OP_MD5) {
		index = (op->byte_count + 4) & 0x3f;
		op->byte_count += op->nbwait;
		if (index > 56)
			zeros = (120 - index) / 4;
		else
			zeros = (56 - index) / 4;
	} else {
		op->byte_count += op->nbwait;
		index = op->byte_count & 0x3f;
		padlen = (index < 56) ? (56 - index) : ((64+56) - index);
		zeros = (padlen - 1) / 4;
	}
	for (i = 0; i < zeros; i++)
		writel(0, ss->base + SS_RXFIFO);

	/* write the lenght */
	if (op->mode == SS_OP_SHA1) {
		bits = cpu_to_be64(op->byte_count << 3);
		writel(bits & 0xffffffff, ss->base + SS_RXFIFO);
		writel((bits >> 32) & 0xffffffff, ss->base + SS_RXFIFO);
	} else {
		writel((op->byte_count << 3) & 0xffffffff,
				ss->base + SS_RXFIFO);
		writel((op->byte_count >> 29) & 0xffffffff,
				ss->base + SS_RXFIFO);
	}

	/* stop the hashing */
	v = readl(ss->base + SS_CTL);
	v |= SS_DATA_END;
	writel(v, ss->base + SS_CTL);

	/* check the end */
	/* The timeout could happend only in case of bad overcloking */
#define SS_TIMEOUT 100
	i = 0;
	do {
		v = readl(ss->base + SS_CTL);
		i++;
	} while (i < SS_TIMEOUT && (v & SS_DATA_END) > 0);
	if (i >= SS_TIMEOUT) {
		dev_err(ss->dev, "ERROR: hash end timeout %d>%d\n",
				i, SS_TIMEOUT);
		writel(0, ss->base + SS_CTL);
		mutex_unlock(&ss->lock);
		return -1;
	}

	if (op->mode == SS_OP_SHA1) {
		for (i = 0; i < 5; i++) {
			v = cpu_to_be32(readl(ss->base + SS_MD0 + i * 4));
			memcpy(areq->result + i * 4, &v, 4);
		}
	} else {
		for (i = 0; i < 4; i++) {
			v = readl(ss->base + SS_MD0 + i * 4);
			memcpy(areq->result + i * 4, &v, 4);
		}
	}
	writel(0, ss->base + SS_CTL);
	mutex_unlock(&ss->lock);
	return 0;
}

/* sunxi_hash_finup: finalize hashing operation after an update */
int sunxi_hash_finup(struct ahash_request *areq)
{
	int err;

	err = sunxi_hash_update(areq);
	if (err != 0)
		return err;

	return sunxi_hash_final(areq);
}

/* combo of init/update/final functions */
int sunxi_hash_digest(struct ahash_request *areq)
{
	int err;

	err = sunxi_hash_init(areq);
	if (err != 0)
		return err;

	err = sunxi_hash_update(areq);
	if (err != 0)
		return err;

	return sunxi_hash_final(areq);
}
