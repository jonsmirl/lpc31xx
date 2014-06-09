#include "sunxi-ss.h"
#include <crypto/internal/rng.h>

#define SS_SEED_LEN (192/8)
#define SS_DATA_LEN (160/8)

struct prng_context {
	u32 seed[SS_SEED_LEN/4];
	unsigned int slen;
};

extern struct sunxi_ss_ctx *ss;

static int sunxi_ss_rng_get_random(struct crypto_rng *tfm, u8 *rdata,
		unsigned int dlen)
{
	struct prng_context *ctx = crypto_rng_ctx(tfm);
	int i;
	u32 mode = 0;
	u32 v;

	if (rdata == NULL)
		return -EINVAL;

	mode |= SS_OP_PRNG | SS_PRNG_ONESHOT | SS_ENABLED;

	mutex_lock(&ss->lock);
	writel(mode, ss->base + SS_CTL);

	for (i = 0; i < ctx->slen && i < SS_SEED_LEN; i += 4) {
		v = *(u32 *)(ctx->seed + i);
		writel(v, ss->base + SS_KEY0 + i);
	}

	writel(mode | SS_PRNG_START, ss->base + SS_CTL);
	i = 0;
	do {
		v = readl(ss->base + SS_CTL);
		i++;
	} while (v != mode && i < 10);
	for (i = 0; i < dlen && i < SS_DATA_LEN; i += 4) {
		v = readl(ss->base + SS_MD0 + i);
		*(u32 *)(rdata + i) = v;
	}

	writel(0, ss->base + SS_CTL);
	mutex_unlock(&ss->lock);
	return dlen;
}

static int sunxi_ss_rng_reset(struct crypto_rng *tfm, u8 *seed,
		unsigned int slen)
{
	struct prng_context *ctx = crypto_rng_ctx(tfm);
	unsigned int i;

	if (slen == 0) {
		ctx->slen = 0;
		return 0;
	}
	if (slen > SS_SEED_LEN) {
		dev_err(ss->dev, "Requested seedlen %u exceed %u\n",
				slen, SS_SEED_LEN);
		return -EINVAL;
	}
	for (i = 0; i < SS_SEED_LEN && i < slen; i++)
		ctx->seed[i] = seed[i];
	ctx->slen = slen;
	return 0;
}

static struct crypto_alg sunxi_ss_prng = {
	.cra_name = "stdrng",
	.cra_driver_name = "rng-sunxi-ss",
	.cra_priority = 300,
	.cra_flags = CRYPTO_ALG_TYPE_RNG,
	.cra_ctxsize = sizeof(struct prng_context),
	.cra_module = THIS_MODULE,
	.cra_type = &crypto_rng_type,
	.cra_u.rng = {
		.rng_make_random = sunxi_ss_rng_get_random,
		.rng_reset = sunxi_ss_rng_reset,
		.seedsize = SS_SEED_LEN
	}
};


static int sunxi_ss_rng_init(void)
{
	int err = 0;
	if (ss == NULL) {
		pr_err("Cannot get SUNXI SS\n");
		return 0;
	}

	err = crypto_register_alg(&sunxi_ss_prng);
	if (err)
		dev_err(ss->dev, "crypto_register_alg error\n");
	else
		dev_info(ss->dev, "Registred PRNG\n");
	return err;
}

static void __exit sunxi_ss_rng_exit(void)
{
	crypto_unregister_alg(&sunxi_ss_prng);
}


module_init(sunxi_ss_rng_init);
module_exit(sunxi_ss_rng_exit);

MODULE_DESCRIPTION("Allwinner Security System crypto accelerator RNG module");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Corentin LABBE <clabbe.montjoie@gmail.com>");
