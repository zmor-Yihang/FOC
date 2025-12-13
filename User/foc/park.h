#ifndef __PARK_H__
#define __PARK_H__

#include <math.h>
#include "./config/type_config.h"
#include "./utils/fast_sin_cos.h"

/**
 * @brief Park变换：将两相静止坐标系(α-β)转换为两相旋转坐标系(D-Q)
 * 
 * 公式：
 * id = iα*cosθ + iβ*sinθ
 * iq = -iα*sinθ + iβ*cosθ
 * 
 * @param input 两相静止坐标系输入 (iα, iβ)
 * @param theta 电角度 (弧度)
 * @param output 两相旋转坐标系输出 (id, iq)
 */
void park_transform(phase2_t input, float theta, phase2_t *output);

/**
 * @brief 反Park变换：将两相旋转坐标系(D-Q)转换为两相静止坐标系(α-β)
 * 
 * 公式矩阵：
 * [Iα]   [cosθ  -sinθ] [Id]
 * [Iβ] = [sinθ   cosθ] [Iq]
 * 
 * @param input 两相旋转坐标系输入 (Id, Iq)
 * @param theta 电角度 (弧度)
 * @param output 两相静止坐标系输出 (Iα, Iβ)
 */
void inverse_park_transform(phase2_t input, float theta, phase2_t *output);

#endif /* __PARK_H__ */