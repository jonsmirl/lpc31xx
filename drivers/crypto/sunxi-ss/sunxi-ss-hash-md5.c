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
#include "sunxi-ss-hash.h"

static struct ahash_alg sunxi_md5_alg = {
	.init = sunxi_hash_init,
	.update = sunxi_hash_update,
	.final = sunxi_hash_final,
	.finup = sunxi_hash_finup,
	.digest = sunxi_hash_digest,
	.halg = {
		.digestsize = MD5_DIGEST_SIZE,
		.base = {
			.cra_name = "md5",
			.cra_driver_name = "md5-sunxi-ss",
			.cra_priority = 300,
			.cra_alignmask = 3,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
			.cra_blocksize = MD5_HMAC_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct sunxi_req_ctx),
			.cra_module = THIS_MODULE,
			.cra_type = &crypto_ahash_type
		}
	}
};

static int sunxi_ss_md5_init(void)
{
	int err = 0;
	if (ss == NULL) {
		pr_err("Cannot get Security System structure\n");
		return -ENODEV;
	}
	err = crypto_register_ahash(&sunxi_md5_alg);
	if (err)
		dev_err(ss->dev, "crypto_register_alg error for MD5\n");
	else
		dev_dbg(ss->dev, "Registred MD5\n");
	return err;
}

static void __exit sunxi_ss_md5_exit(void)
{
	crypto_unregister_ahash(&sunxi_md5_alg);
}

module_init(sunxi_ss_md5_init);
module_exit(sunxi_ss_md5_exit);

MODULE_DESCRIPTION("Allwinner Security System crypto accelerator MD5 module");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Corentin LABBE <clabbe.montjoie@gmail.com>");
MODULE_ALIAS("md5");
