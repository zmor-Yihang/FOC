#ifndef __MATH_UTILS_H__
#define __MATH_UTILS_H__

#include <math.h>

#define TWO_PI (2.0f * M_PI)

/**
 * @brief 角度归一化到 [0, 2π)
 * @param angle 输入角度 (rad)
 * @return 归一化后的角度
 */
static inline float normalize_angle(float angle)
{
    while (angle >= TWO_PI)
        angle -= TWO_PI;
    while (angle < 0.0f)
        angle += TWO_PI;
    return angle;
}

#endif /* __MATH_UTILS_H__ */
