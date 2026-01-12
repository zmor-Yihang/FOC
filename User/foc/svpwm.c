#include "svpwm.h"

/**
 * @brief  标准七段式SVPWM (扇区法)
 * @param  u_alphabeta - αβ轴电压 (V)
 * @retval duty - 输出的三相占空比 (范围 0.0 ~ 1.0)
 * @note   占空比围绕0.5中心分布
 */
abc_t svpwm_sector1(alphabeta_t u_alphabeta)
{
    abc_t duty;
    float u1, u2, u3;
    float t1, t2, t0_half;
    float t_sum, t_max, t_mid, t_min;

    float v_alpha = u_alphabeta.alpha;
    float v_beta = u_alphabeta.beta;
    float sqrt3_v_alpha = SQRT3 * v_alpha;

    /* 计算参考电压矢量在三个轴上的投影 */
    u1 = v_beta;
    u2 = (sqrt3_v_alpha - v_beta) * 0.5f;
    u3 = (-sqrt3_v_alpha - v_beta) * 0.5f;

    /* 扇区判断 + 矢量作用时间计算 (合并处理) */
    int sector = (u1 > 0) + ((u2 > 0) << 1) + ((u3 > 0) << 2);

    switch (sector)
    {
    case 3:
        t1 = u2;
        t2 = u1;
        break;
    case 1:
        t1 = -u3;
        t2 = -u2;
        break;
    case 5:
        t1 = u1;
        t2 = u3;
        break;
    case 4:
        t1 = -u2;
        t2 = -u1;
        break;
    case 6:
        t1 = u3;
        t2 = u2;
        break;
    case 2:
        t1 = -u1;
        t2 = -u3;
        break;
    default:
        t1 = 0;
        t2 = 0;
        break;
    }

    /* 归一化 */
    float k_norm = SQRT3 / U_DC;
    t1 *= k_norm;
    t2 *= k_norm;

    /* 过调制处理 */
    t_sum = t1 + t2;
    if (t_sum > 1.0f)
    {
        float k = 1.0f / t_sum;
        t1 *= k;
        t2 *= k;
        t_sum = 1.0f;
    }

    /* 预计算公共项 */
    t0_half = (1.0f - t_sum) * 0.5f;
    t_max = t_sum + t0_half; /* t1 + t2 + t0/2 */
    t_mid = t2 + t0_half;    /* t2 + t0/2 */
    t_min = t0_half;         /* t0/2 */

    /* 根据扇区分配占空比 */
    switch (sector)
    {
    case 3:
        duty.a = t_max;
        duty.b = t_mid;
        duty.c = t_min;
        break;
    case 1:
        duty.a = t_mid;
        duty.b = t_max;
        duty.c = t_min;
        break;
    case 5:
        duty.a = t_min;
        duty.b = t_max;
        duty.c = t_mid;
        break;
    case 4:
        duty.a = t_min;
        duty.b = t_mid;
        duty.c = t_max;
        break;
    case 6:
        duty.a = t_mid;
        duty.b = t_min;
        duty.c = t_max;
        break;
    case 2:
        duty.a = t_max;
        duty.b = t_min;
        duty.c = t_mid;
        break;
    default:
        duty.a = duty.b = duty.c = 0.5f;
        break;
    }

    return duty;
}

/**
 * @brief  标准SVPWM调制函数 (教科书风格，优化版)
 * @param  u_alphabeta - αβ轴电压 (V)
 * @retval duty - 输出的三相占空比 (范围 0.0 ~ 1.0)
 * @note   参考《现代永磁同步电机控制原理及MATLAB仿真》 2.4.2节
 */
abc_t svpwm_sector2(alphabeta_t u_alphabeta)
{
    abc_t duty;
    int32_t sector = 0;
    float Tx, Ty, T0_half;
    float T_max, T_mid, T_min;

    float v_alpha = u_alphabeta.alpha;
    float v_beta = u_alphabeta.beta;

    /* 预计算公共项 */
    float sqrt3_v_beta = SQRT3 * v_beta;
    float term1 = (1.5f * v_alpha + SQRT3_DIV2 * v_beta) / U_DC;
    float term2 = (-1.5f * v_alpha + SQRT3_DIV2 * v_beta) / U_DC;
    float term3 = sqrt3_v_beta / U_DC;

    /* 扇区判断 */
    if (v_beta > 0.0f)
        sector = 1;
    if ((SQRT3 * v_alpha - v_beta) > 0.0f)
        sector += 2;
    if ((-SQRT3 * v_alpha - v_beta) > 0.0f)
        sector += 4;

    /* 计算矢量作用时间 (复用预计算项) */
    switch (sector)
    {
    case 1:
        Tx = term2;
        Ty = term1;
        break;
    case 2:
        Tx = term1;
        Ty = -term3;
        break;
    case 3:
        Tx = -term2;
        Ty = term3;
        break;
    case 4:
        Tx = -term3;
        Ty = term2;
        break;
    case 5:
        Tx = term3;
        Ty = -term1;
        break;
    default:
        Tx = -term1;
        Ty = -term2;
        break; /* sector 6 */
    }

    /* 过调制处理 */
    float t_sum = Tx + Ty;
    if (t_sum > 1.0f)
    {
        float k = 1.0f / t_sum;
        Tx *= k;
        Ty *= k;
        t_sum = 1.0f;
    }

    /* 预计算公共项 */
    T0_half = (1.0f - t_sum) * 0.5f;
    T_max = t_sum + T0_half;
    T_mid = Ty + T0_half;
    T_min = T0_half;

    /* 根据扇区分配占空比 */
    switch (sector)
    {
    case 1:
        duty.a = T_mid;
        duty.b = T_min;
        duty.c = T_max;
        break;
    case 2:
        duty.a = T_min;
        duty.b = T_max;
        duty.c = T_mid;
        break;
    case 3:
        duty.a = T_min;
        duty.b = T_mid;
        duty.c = T_max;
        break;
    case 4:
        duty.a = T_max;
        duty.b = T_mid;
        duty.c = T_min;
        break;
    case 5:
        duty.a = T_max;
        duty.b = T_min;
        duty.c = T_mid;
        break;
    case 6:
        duty.a = T_mid;
        duty.b = T_max;
        duty.c = T_min;
        break;
    default:
        duty.a = duty.b = duty.c = 0.5f;
        break;
    }

    return duty;
}

/**
 * @brief  SVPWM调制函数 (min-max零序注入法)
 * @param  u_alphabeta - αβ轴电压 (V)
 * @retval duty - 输出的三相占空比 (范围 0.0 ~ 1.0)
 */
abc_t svpwm_minmax(alphabeta_t u_alphabeta)
{
    abc_t duty;
    float u_max, u_min, u_zero;
    float inv_half_udc = 1.0f / (U_DC * 0.5f);

    /* 反Clark变换: αβ -> abc */
    abc_t u_abc = iclark_transform(u_alphabeta);

    /* 归一化到 [-1, 1] */
    u_abc.a *= inv_half_udc;
    u_abc.b *= inv_half_udc;
    u_abc.c *= inv_half_udc;

    /* 找最大最小值 */
    if (u_abc.a > u_abc.b)
    {
        u_max = (u_abc.a > u_abc.c) ? u_abc.a : u_abc.c;
        u_min = (u_abc.b < u_abc.c) ? u_abc.b : u_abc.c;
    }
    else
    {
        u_max = (u_abc.b > u_abc.c) ? u_abc.b : u_abc.c;
        u_min = (u_abc.a < u_abc.c) ? u_abc.a : u_abc.c;
    }

    /* 计算零序分量并注入 */
    u_zero = -0.5f * (u_max + u_min);

    /* 映射到占空比 [0, 1] */
    duty.a = (u_abc.a + u_zero) * 0.5f + 0.5f;
    duty.b = (u_abc.b + u_zero) * 0.5f + 0.5f;
    duty.c = (u_abc.c + u_zero) * 0.5f + 0.5f;

    /* 占空比限幅 */
    duty.a = (duty.a > 1.0f) ? 1.0f : ((duty.a < 0.0f) ? 0.0f : duty.a);
    duty.b = (duty.b > 1.0f) ? 1.0f : ((duty.b < 0.0f) ? 0.0f : duty.b);
    duty.c = (duty.c > 1.0f) ? 1.0f : ((duty.c < 0.0f) ? 0.0f : duty.c);

    return duty;
}

/* 默认使用扇区法 */
abc_t svpwm_update(alphabeta_t u_alphabeta)
{
    return svpwm_sector(u_alphabeta);
}
