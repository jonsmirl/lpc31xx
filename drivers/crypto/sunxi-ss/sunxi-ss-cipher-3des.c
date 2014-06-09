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

/* check and set the 3DES key, prepare the mode to be used */
static int sunxi_des3_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
		unsigned int keylen)
{
	struct sunxi_req_ctx *op = crypto_ablkcipher_ctx(tfm);
	if (keylen != 3 * DES_KEY_SIZE) {
		dev_err(ss->dev, "Invalid keylen %u\n", keylen);
		crypto_ablkcipher_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}
	op->keylen = keylen;
	memcpy(op->key, key, keylen);
	return 0;
}

static int sunxi_des3_cbc_encrypt(struct ablkcipher_request *areq)
{
	struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(areq);
	struct sunxi_req_ctx *op = crypto_ablkcipher_ctx(tfm);

	if (areq->info == NULL) {
		dev_info(ss->dev, "Empty IV\n");
		return -EINVAL;
	}

	op->mode |= SS_ENCRYPTION;
	op->mode |= SS_OP_3DES;
	op->mode |= SS_CBC;

	return sunxi_des_poll(areq);
}

static int sunxi_des3_cbc_decrypt(struct ablkcipher_request *areq)
{
	struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(areq);
	struct sunxi_req_ctx *op = crypto_ablkcipher_ctx(tfm);

	if (areq->info == NULL) {
		dev_info(ss->dev, "Empty IV\n");
		return -EINVAL;
	}

	op->mode |= SS_DECRYPTION;
	op->mode |= SS_OP_3DES;
	op->mode |= SS_CBC;

	return sunxi_des_poll(areq);
}

static struct crypto_alg sunxi_des3_alg = {
	.cra_name = "cbc(des3_ede)",
	.cra_driver_name = "cbc-des3-sunxi-ss",
	.cra_priority = 300,
	.cra_blocksize = DES3_EDE_BLOCK_SIZE,
	.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER,
	.cra_ctxsize = sizeof(struct sunxi_req_ctx),
	.cra_module = THIS_MODULE,
	.cra_type = &crypto_ablkcipher_type,
	.cra_init = sunxi_cipher_init,
	.cra_exit = sunxi_cipher_exit,
	.cra_alignmask = 3,
	.cra_u.ablkcipher = {
		.min_keysize    = DES3_EDE_KEY_SIZE,
		.max_keysize    = DES3_EDE_KEY_SIZE,
		.ivsize         = DES3_EDE_BLOCK_SIZE,
		.setkey         = sunxi_des3_setkey,
		.encrypt        = sunxi_des3_cbc_encrypt,
		.decrypt        = sunxi_des3_cbc_decrypt,
	}
};

static int sunxi_ss_3des_init(void)
{
	int err = 0;
	if (ss == NULL) {
		pr_err("Cannot get Security System structure\n");
		return -ENODEV;
	}
	err = crypto_register_alg(&sunxi_des3_alg);
	if (err)
		dev_err(ss->dev, "crypto_register_alg error for DES3\n");
	else
		dev_dbg(ss->dev, "Registred DES3\n");
	return err;
}

static void __exit sunxi_ss_3des_exit(void)
{
	crypto_unregister_alg(&sunxi_des3_alg);
}

module_init(sunxi_ss_3des_init);
module_exit(sunxi_ss_3des_exit);

MODULE_DESCRIPTION("Allwinner Security System crypto accelerator DES3 module");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Corentin LABBE <clabbe.montjoie@gmail.com>");
MODULE_ALIAS("3des");
