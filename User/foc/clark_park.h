#ifndef __CLARK_PARK_H__
#define __CLARK_PARK_H__

#include "./config/type_config.h"
#include "./utils/fast_sin_cos.h"

alphabeta_t clark_transform(abc_t abc);
abc_t iclark_transform(alphabeta_t alpha_beta);
dq_t park_transform(alphabeta_t alpha_beta, float theta);
alphabeta_t ipark_transform(dq_t dq, float theta);

#endif /* __CLARK_PARK_H__ */