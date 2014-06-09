#include "sunxi-ss.h"

extern struct sunxi_ss_ctx *ss;

int sunxi_aes_poll(struct ablkcipher_request *areq);
int sunxi_des_poll(struct ablkcipher_request *areq);
int sunxi_cipher_init(struct crypto_tfm *tfm);
void sunxi_cipher_exit(struct crypto_tfm *tfm);
