#include "clark_park.h"

alphabeta_t clark_transform(abc_t abc)
{
    alphabeta_t alpha_beta;

    float one_by_sqrt3 = 1.0f / sqrtf(3.0f);

    // iα = Ia
    alpha_beta.alpha = abc.a;

    // iβ = (1/√3)Ia + (2/√3)Ib
    alpha_beta.beta = one_by_sqrt3 * abc.a + 2.0f * one_by_sqrt3 * abc.b;

    return alpha_beta;
}

abc_t iclark_transform(alphabeta_t alpha_beta)
{
    abc_t abc;

    float sqrt3_by_2 = sqrtf(3.0f) / 2.0f;

    // Ia = iα
    abc.a = alpha_beta.alpha;

    // Ib = -iα/2 + (√3/2)iβ
    abc.b = -0.5f * alpha_beta.alpha + sqrt3_by_2 * alpha_beta.beta;

    // Ic = -Ia - Ib
    abc.c = -abc.a - abc.b;

    return abc;
}

dq_t park_transform(alphabeta_t alpha_beta, float theta)
{
    dq_t dq;

    float sin_theta, cos_theta;

    // 使用快速正弦余弦函数计算sinθ和cosθ
    fast_sin_cos(theta, &sin_theta, &cos_theta);

    // id = iα*cosθ + iβ*sinθ
    dq.d = alpha_beta.alpha * cos_theta + alpha_beta.beta * sin_theta;

    // iq = -iα*sinθ + iβ*cosθ
    dq.q = -alpha_beta.alpha * sin_theta + alpha_beta.beta * cos_theta;

    return dq;
}

alphabeta_t ipark_transform(dq_t dq, float theta)
{
    alphabeta_t alpha_beta;

    float sin_theta, cos_theta;

    // 快速正弦余弦函数计算sinθ和cosθ
    fast_sin_cos(theta, &sin_theta, &cos_theta);

    // 反Park变换公式实现
    // Iα = Id*cosθ - Iq*sinθ
    alpha_beta.alpha = dq.d * cos_theta - dq.q * sin_theta;

    // Iβ = Id*sinθ + Iq*cosθ
    alpha_beta.beta = dq.d * sin_theta + dq.q * cos_theta;

    return alpha_beta;
}
