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

static int sunxi_aes_cbc_encrypt(struct ablkcipher_request *areq)
{
	struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(areq);
	struct sunxi_req_ctx *op = crypto_ablkcipher_ctx(tfm);

	if (areq->info == NULL) {
		dev_err(ss->dev, "Empty IV\n");
		return -EINVAL;
	}

	op->mode |= SS_ENCRYPTION;
	op->mode |= SS_OP_AES;
	op->mode |= SS_CBC;

	return sunxi_aes_poll(areq);
}

static int sunxi_aes_cbc_decrypt(struct ablkcipher_request *areq)
{
	struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(areq);
	struct sunxi_req_ctx *op = crypto_ablkcipher_ctx(tfm);

	if (areq->info == NULL) {
		dev_err(ss->dev, "Empty IV\n");
		return -EINVAL;
	}

	op->mode |= SS_DECRYPTION;
	op->mode |= SS_OP_AES;
	op->mode |= SS_CBC;

	return sunxi_aes_poll(areq);
}

/* check and set the AES key, prepare the mode to be used */
static int sunxi_aes_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
		unsigned int keylen)
{
	struct sunxi_req_ctx *op = crypto_ablkcipher_ctx(tfm);
	switch (keylen) {
	case 128 / 8:
		op->mode = SS_AES_128BITS;
		break;
	case 192 / 8:
		op->mode = SS_AES_192BITS;
		break;
	case 256 / 8:
		op->mode = SS_AES_256BITS;
		break;
	default:
		dev_err(ss->dev, "Invalid keylen %u\n", keylen);
		crypto_ablkcipher_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}
	op->keylen = keylen;
	memcpy(op->key, key, keylen);
	return 0;
}

static struct crypto_alg sunxi_aes_alg = {
	.cra_name = "cbc(aes)",
	.cra_driver_name = "cbc-aes-sunxi-ss",
	.cra_priority = 300,
	.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
	.cra_blocksize   = AES_BLOCK_SIZE,
	.cra_ctxsize = sizeof(struct sunxi_req_ctx),
	.cra_module = THIS_MODULE,
	.cra_alignmask = 3,
	.cra_type = &crypto_ablkcipher_type,
	.cra_init = sunxi_cipher_init,
	.cra_exit = sunxi_cipher_exit,
	.cra_u = {
		.ablkcipher = {
			.min_keysize    = AES_MIN_KEY_SIZE,
			.max_keysize    = AES_MAX_KEY_SIZE,
			.ivsize         = AES_BLOCK_SIZE,
			.setkey         = sunxi_aes_setkey,
			.encrypt        = sunxi_aes_cbc_encrypt,
			.decrypt        = sunxi_aes_cbc_decrypt,
		}
	}
};

int sunxi_ss_aes_init(void)
{
	int err = 0;
	if (ss == NULL) {
		pr_err("Cannot get Security System structure\n");
		return -ENODEV;
	}
	err = crypto_register_alg(&sunxi_aes_alg);
	if (err)
		dev_err(ss->dev, "crypto_register_alg error for AES\n");
	else
		dev_dbg(ss->dev, "Registred AES\n");
	return err;
}

static void __exit sunxi_ss_aes_exit(void)
{
	crypto_unregister_alg(&sunxi_aes_alg);
}

module_init(sunxi_ss_aes_init);
module_exit(sunxi_ss_aes_exit);

MODULE_DESCRIPTION("Allwinner Security System crypto accelerator AES module");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Corentin LABBE <clabbe.montjoie@gmail.com>");
MODULE_ALIAS("aes");
