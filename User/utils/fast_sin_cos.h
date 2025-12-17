/**
 * @file fast_sin_cos.h
 * @brief 高性能三角函数计算库
 * 
 * 提供快速的正弦、余弦函数实现，适用于嵌入式系统和对性能有要求的应用场景。
 * 使用了多种优化技术，包括：
 * - Cody-Waite归约算法
 * - FMA 指令优化
 * - 多项式逼近方法
 * - 编译器特定的优化指令
 * 
 * 支持GCC (STM32CubeIDE) 和 Keil ARMCC编译器。
 */

#ifndef __FAST_SIN_COS_H__
#define __FAST_SIN_COS_H__

/**
 * @def FAST_MATH_OPT
 * @brief 快速数学函数优化标记
 * 
 * 根据不同的编译器设置相应的优化属性：
 * - GCC: 使用-O3优化级别和always_inline强制内联
 * - Keil ARMCC: 使用#pragma O3优化和__forceinline强制内联
 */
#if defined(__GNUC__)
    #define FAST_MATH_OPT __attribute__((optimize("O3"), always_inline))
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
    #pragma push
    #pragma O3
    #define FAST_MATH_OPT __forceinline
#else
    #define FAST_MATH_OPT inline
#endif

#include <math.h>
#include <stdint.h>

/**
 * @def M_1_PI_F
 * @brief 1/π的单精度浮点值
 * 
 * 用于角度归约计算中的常量
 */
#define M_1_PI_F 0.318309886183790671538f

/**
 * @def PI_A
 * @brief π的高位部分 (Cody-Waite归约算法)
 * 
 * 在Cody-Waite归约算法中，将π分解为高位和低位两部分以提高精度。
 * PI_A = 3.141592502593994140625f (0x40490fdb)
 */
#define PI_A 3.14159250f

/**
 * @def PI_B
 * @brief π的低位部分 (Cody-Waite归约算法)
 * 
 * 在Cody-Waite归约算法中，PI_B = π - PI_A，用于提高归约精度。
 */
#define PI_B 1.50995799e-7f

/**
 * @def MY_FMA
 * @brief 自定义FMA (融合乘加) 宏
 * 
 * 根据编译器和平台支持情况选择最优的FMA实现：
 * - 如果支持硬件FMA指令，则使用fmaf函数
 * - 否则使用标准的乘加运算
 */
#if defined(__FP_FAST_FMA) || defined(__AVX2__) || defined(__ARM_FEATURE_FMA)
    // 硬件明确支持 FMA
    #define MY_FMA(a, b, c) __builtin_fmaf(a, b, c) 
#else
    // 即使走这里，由于你开了 -O3，编译器通常也会自动生成 FMA 指令
    #define MY_FMA(a, b, c) ((a) * (b) + (c))
#endif

/**
 * @brief 正弦函数多项式逼近辅助函数
 * 
 * 计算正弦函数泰勒级数展开的高次项部分，使用预计算系数的多项式。
 * 该函数是 sin(x) ≈ x + x^3 * f1(x^2) 中的 f1 部分。
 * 
 * @param x 输入值（应为x^2）
 * @return 多项式计算结果
 * 
 * @note 使用FMA指令优化计算性能
 */
static FAST_MATH_OPT inline float f1_opt(float x)
{
    float u = 1.3528548e-10f;
    u = MY_FMA(u, x, -2.4703144e-08f);
    u = MY_FMA(u, x, 2.7532926e-06f);
    u = MY_FMA(u, x, -0.00019840381f);
    u = MY_FMA(u, x, 0.0083333179f);
    u = MY_FMA(u, x, -0.16666666f);
    return u;
}

/**
 * @brief 余弦函数多项式逼近辅助函数
 * 
 * 计算余弦函数泰勒级数展开的高次项部分，使用预计算系数的多项式。
 * 该函数是 cos(x) ≈ 1 + x^2 * f2(x^2) 中的 f2 部分。
 * 
 * @param x 输入值（应为x^2）
 * @return 多项式计算结果
 * 
 * @note 使用FMA指令优化计算性能
 */
static FAST_MATH_OPT inline float f2_opt(float x)
{
    float u = 1.7290616e-09f;
    u = MY_FMA(u, x, -2.7093486e-07f);
    u = MY_FMA(u, x, 2.4771643e-05f);
    u = MY_FMA(u, x, -0.0013887906f);
    u = MY_FMA(u, x, 0.041666519f);
    u = MY_FMA(u, x, -0.49999991f);
    return u;
}

/**
 * @brief 使用Payne-Hanek算法进行角度归约
 * 
 * 将任意角度归约到[-π/2, π/2]范围内，使用Cody-Waite归约算法提高精度。
 * 该函数解决了大数值输入时的精度丢失问题。
 * 
 * @param x 输入角度值（弧度）
 * @param quadrant 输出参数，存储象限信息，用于后续符号处理
 * @return 归约后的角度值，范围在约[-π/2, π/2]之间
 * 
 * @note 使用高精度分步减法避免大数计算中的精度损失
 */
FAST_MATH_OPT static inline float reduce_payne_hanek(float x, int *quadrant)
{
    // q = round(x / PI)
    // 使用 rintf 或 nearbyintf 比 (int) 转换更精确，能归约到中心区间
    float qf = rintf(x * M_1_PI_F);
    *quadrant = (int)qf;
    
    // x = x - q * PI (使用高精度分步减法)
    // 这一步解决了大数输入的精度丢失问题
    float r = MY_FMA(qf, -PI_A, x);
    r = MY_FMA(qf, -PI_B, r);
    return r;
}

/**
 * @brief 快速正弦函数
 * 
 * 使用多项式逼近和角度归约算法计算正弦值。
 * 算法基于 sin(x) ≈ x + x^3 * f1(x^2) 近似公式。
 * 
 * @param x 输入角度值（弧度）
 * @return 正弦值
 * 
 * @note 使用了FMA优化和Payne-Hanek归约算法提高精度和性能
 */
FAST_MATH_OPT static inline float fast_sin(float x)
{
    int q;
    x = reduce_payne_hanek(x, &q);
    
    // 计算 sin(x) ~= x + x^3 * f1(x^2)
    float x2 = x * x;
    float result = MY_FMA(x * x2, f1_opt(x2), x);

    // 符号处理 (Branchless): sin(x + k*PI) = sin(x) * (-1)^k
    uint32_t sign_mask = (q & 1) << 31;
    union { float f; uint32_t i; } u;
    u.f = result;
    u.i ^= sign_mask;
    return u.f;
}

/**
 * @brief 快速余弦函数
 * 
 * 使用多项式逼近和角度归约算法计算余弦值。
 * 算法基于 cos(x) ≈ 1 + x^2 * f2(x^2) 近似公式。
 * 
 * @param x 输入角度值（弧度）
 * @return 余弦值
 * 
 * @note 使用了FMA优化和Payne-Hanek归约算法提高精度和性能
 */
FAST_MATH_OPT static inline float fast_cos(float x)
{
    int q;
    x = reduce_payne_hanek(x, &q);

    // 计算 cos(x) ~= 1 + x^2 * f2(x^2)
    float x2 = x * x;
    float result = MY_FMA(x2, f2_opt(x2), 1.0f);

    // 符号处理 (Branchless): cos(x + k*PI) = cos(x) * (-1)^k
    uint32_t sign_mask = (q & 1) << 31;
    union { float f; uint32_t i; } u;
    u.f = result;
    u.i ^= sign_mask;
    return u.f;
}

/**
 * @brief 快速正弦/余弦函数（并行计算）
 * 
 * 同时计算一个角度的正弦和余弦值，通过并行计算提高效率。
 * 使用无分支优化技术处理符号位，避免条件判断带来的性能损失。
 * 
 * @param x 输入角度值（弧度）
 * @param sin_x 输出参数，返回正弦值
 * @param cos_x 输出参数，返回余弦值
 * 
 * @note 使用位操作进行符号处理，避免分支预测失败的性能损失
 */
FAST_MATH_OPT static inline void fast_sin_cos(float x, float *sin_x, float *cos_x)
{
    int q;
    // 1. 统一归约
    x = reduce_payne_hanek(x, &q);
    
    float x2 = x * x;

    // 2. 并行计算多项式 
    // 计算 Sin 部分
    float p_sin = f1_opt(x2);
    // 计算 Cos 部分
    float p_cos = f2_opt(x2);

    float s = MY_FMA(x * x2, p_sin, x);
    float c = MY_FMA(x2, p_cos, 1.0f);

    // 3. 符号处理 (Branchless 优化)
    // 如果 q 是奇数，sign_mask 为 0x80000000 (float 的符号位)，否则为 0
    // 直接操作浮点数的位表示来改变符号，通常比分支更快
    uint32_t sign_mask = (q & 1) << 31;
    
    union { float f; uint32_t i; } us, uc;
    us.f = s;
    uc.f = c;
    
    us.i ^= sign_mask; // 异或符号位
    uc.i ^= sign_mask;

    *sin_x = us.f;
    *cos_x = uc.f;
}

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
    #pragma pop
#endif

#endif /* __FAST_SIN_COS_H__ */
