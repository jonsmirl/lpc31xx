#include "sunxi-ss.h"

extern struct sunxi_ss_ctx *ss;

int sunxi_hash_init(struct ahash_request *areq);
int sunxi_hash_update(struct ahash_request *areq);
int sunxi_hash_final(struct ahash_request *areq);
int sunxi_hash_finup(struct ahash_request *areq);
int sunxi_hash_digest(struct ahash_request *areq);
