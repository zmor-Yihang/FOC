#ifndef __CLARK_PARK_H__
#define __CLARK_PARK_H__

#include "stm32g4xx_hal.h"
#include "./utils/fast_sin_cos.h"

/* 三相坐标系 */
typedef struct
{
    float a; /* U */
    float b; /* V */
    float c; /* W */
} abc_t;

/* 静止坐标系 */
typedef struct
{
    float alpha;
    float beta;
} alphabeta_t;

/* 旋转坐标系 */
typedef struct
{
    float d;
    float q;
} dq_t;

alphabeta_t clark_transform(abc_t abc);
abc_t iclark_transform(alphabeta_t alpha_beta);
dq_t park_transform(alphabeta_t alpha_beta, float theta);
alphabeta_t ipark_transform(dq_t dq, float theta);

#endif /* __CLARK_PARK_H__ */