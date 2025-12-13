#ifndef __CLARK_H__
#define __CLARK_H__

#include <math.h>
#include "./config/type_config.h"
#include "./utils/fast_sin_cos.h"

/**
 * @brief Clarke变换：将三相静止坐标系(U-V-W)转换为两相静止坐标系(α-β)
 *
 * 公式：
 * iα = Ia
 * iβ = (1/√3)Ia + (2/√3)Ib
 *
 * @param input 三相静止坐标系输入 (Ia, Ib, Ic)
 * @param output 两相静止坐标系输出 (iα, iβ)
 */
void clark_transform(phase3_t input, phase2_t *output);

/**
 * @brief 反Clarke变换：将两相静止坐标系(α-β)转换为三相静止坐标系(U-V-W)
 * 
 * 公式：
 * Ia = iα
 * Ib = -iα/2 + (√3/2)iβ
 * Ic = -iα/2 - (√3/2)iβ = -iα - iβ
 *
 * @param input 两相静止坐标系输入 (iα, iβ)
 * @param output 三相静止坐标系输出 (Ia, Ib, Ic)
 */
void iclark_transform(phase2_t input, phase3_t *output);

#endif /* __CLARK_H__ */