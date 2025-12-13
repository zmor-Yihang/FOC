#include "park.h"

void park_transform(phase2_t input, float theta, phase2_t *output)
{
    float sin_theta, cos_theta;
    
    // 使用快速正弦余弦函数计算sinθ和cosθ
    fast_sin_cos(theta, &sin_theta, &cos_theta);
    
    // Park变换公式实现
    // id = iα*cosθ + iβ*sinθ
    output->axis_1 = input.axis_1 * cos_theta + input.axis_2 * sin_theta;
    
    // iq = -iα*sinθ + iβ*cosθ
    output->axis_2 = -input.axis_1 * sin_theta + input.axis_2 * cos_theta;
}

void inverse_park_transform(phase2_t input, float theta, phase2_t *output)
{
    float sin_theta, cos_theta;
    
    // 使用快速正弦余弦函数计算sinθ和cosθ
    fast_sin_cos(theta, &sin_theta, &cos_theta);
    
    // 反Park变换公式实现
    // Iα = Id*cosθ - Iq*sinθ
    output->axis_1 = input.axis_1 * cos_theta - input.axis_2 * sin_theta;
    
    // Iβ = Id*sinθ + Iq*cosθ
    output->axis_2 = input.axis_1 * sin_theta + input.axis_2 * cos_theta;
}