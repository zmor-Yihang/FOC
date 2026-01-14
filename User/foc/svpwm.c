#include "svpwm.h"

/**
 * @brief  标准七段式SVPWM (扇区法)
 * @param  u_alphabeta - αβ轴电压 (V)
 * @retval duty - 输出的三相占空比 (范围 0.0 ~ 1.0)
 * @note   占空比围绕0.25中心分布，与min-max注入法输出一致
 */
abc_t svpwm_sector1(alphabeta_t u_alphabeta)
{
    abc_t duty;
    int N;
    float u1, u2, u3;
    float t1, t2, t0_half;

    float v_alpha = u_alphabeta.alpha;
    float v_beta = u_alphabeta.beta;

    /* 计算参考电压矢量在三个轴上的投影 */
    u1 = v_beta;
    u2 = (1.732051f * v_alpha - v_beta) * 0.5f;
    u3 = (-1.732051f * v_alpha - v_beta) * 0.5f;

    /* 扇区判断 (N = 4*sign(u3) + 2*sign(u2) + sign(u1)) */
    N = (u1 > 0) + ((u2 > 0) << 1) + ((u3 > 0) << 2);

    /* 根据扇区计算矢量作用时间 */
    switch (N)
    {
    case 3: /* 扇区1 */
        t1 = u2;
        t2 = u1;
        break;
    case 1: /* 扇区2 */
        t1 = -u3;
        t2 = -u2;
        break;
    case 5: /* 扇区3 */
        t1 = u1;
        t2 = u3;
        break;
    case 4: /* 扇区4 */
        t1 = -u2;
        t2 = -u1;
        break;
    case 6: /* 扇区5 */
        t1 = u3;
        t2 = u2;
        break;
    case 2: /* 扇区6 */
        t1 = -u1;
        t2 = -u3;
        break;
    default:
        t1 = 0;
        t2 = 0;
        break;
    }

    /* 归一化时间 */
    t1 = t1 * 1.732051f / U_DC;
    t2 = t2 * 1.732051f / U_DC;

    /* 过调制处理 */
    if ((t1 + t2) > 1.0f)
    {
        float k = 1.0f / (t1 + t2);
        t1 *= k;
        t2 *= k;
    }

    t0_half = (1.0f - t1 - t2) * 0.5f;

    /* 计算三相占空比 (中心对称分布，与min-max注入法一致) */
    switch (N)
    {
    case 3: /* 扇区1 */
        duty.a = t1 + t2 + t0_half;
        duty.b = t2 + t0_half;
        duty.c = t0_half;
        break;
    case 1: /* 扇区2 */
        duty.a = t1 + t0_half;
        duty.b = t1 + t2 + t0_half;
        duty.c = t0_half;
        break;
    case 5: /* 扇区3 */
        duty.a = t0_half;
        duty.b = t1 + t2 + t0_half;
        duty.c = t2 + t0_half;
        break;
    case 4: /* 扇区4 */
        duty.a = t0_half;
        duty.b = t1 + t0_half;
        duty.c = t1 + t2 + t0_half;
        break;
    case 6: /* 扇区5 */
        duty.a = t2 + t0_half;
        duty.b = t0_half;
        duty.c = t1 + t2 + t0_half;
        break;
    case 2: /* 扇区6 */
        duty.a = t1 + t2 + t0_half;
        duty.b = t0_half;
        duty.c = t1 + t0_half;
        break;
    default:
        duty.a = duty.b = duty.c = 0.5f;
        break;
    }

    return duty;
}

/**
 * @brief  标准SVPWM调制函数
 * @param  u_alphabeta - αβ轴电压 (V)
 * @retval duty - 输出的三相占空比 (范围 0.0 ~ 1.0)
 * @note   参考《现代永磁同步电机控制原理及MATLAB仿真》 2.4.2节
 */
abc_t svpwm_sector2(alphabeta_t u_alphabeta)
{
    abc_t duty;
    int32_t N = 0, sector = 0;
    float Tx = 0.0f, Ty = 0.0f;

    float v_alpha = u_alphabeta.alpha;
    float v_beta = u_alphabeta.beta;

    /* 扇区判断 */
    if (v_beta > 0.0f)
        N = 1;
    if ((1.732051f * v_alpha - v_beta) > 0.0f)
        N += 2;
    if ((-1.732051f * v_alpha - v_beta) > 0.0f)
        N += 4;

    switch (N)
    {
    case 3:
        sector = 1;
        break;
    case 1:
        sector = 2;
        break;
    case 5:
        sector = 3;
        break;
    case 4:
        sector = 4;
        break;
    case 6:
        sector = 5;
        break;
    case 2:
        sector = 6;
        break;
    }

    /* 预计算公共项 */
    float X = 1.732051f * v_beta / U_DC;
    float Y = (1.5f * v_alpha - 0.866025f * v_beta) / U_DC;
    float Z = (-1.5f * v_alpha - 0.866025f * v_beta) / U_DC;

    /* 计算矢量作用时间 (复用预计算项) */
    switch (sector)
    {
    case 1:
        Tx = Y;
        Ty = X;
        break;
    case 2:
        Tx = -Z;
        Ty = -Y;
        break;
    case 3:
        Tx = X;
        Ty = Z;
        break;
    case 4:
        Tx = -Y;
        Ty = -X;
        break;
    case 5:
        Tx = Z;
        Ty = Y;
        break;
    case 6:
        Tx = -X;
        Ty = -Z;
        break;
    }

    /* 过调制处理 */
    if ((Tx + Ty) > 1.0f)
    {
        float k = 1.0f / (Tx + Ty);
        Tx *= k;
        Ty *= k;
    }

    /* 计算零矢量时间的一半 */
    float t0_half = (1.0f - Tx - Ty) * 0.5f;

    /* 根据扇区分配占空比 (中心对称分布，与sector1一致) */
    switch (sector)
    {
    case 1:
        duty.a = Tx + Ty + t0_half;
        duty.b = Ty + t0_half;
        duty.c = t0_half;
        break;
    case 2:
        duty.a = Tx + t0_half;
        duty.b = Tx + Ty + t0_half;
        duty.c = t0_half;
        break;
    case 3:
        duty.a = t0_half;
        duty.b = Tx + Ty + t0_half;
        duty.c = Ty + t0_half;
        break;
    case 4:
        duty.a = t0_half;
        duty.b = Tx + t0_half;
        duty.c = Tx + Ty + t0_half;
        break;
    case 5:
        duty.a = Ty + t0_half;
        duty.b = t0_half;
        duty.c = Tx + Ty + t0_half;
        break;
    case 6:
        duty.a = Tx + Ty + t0_half;
        duty.b = t0_half;
        duty.c = Tx + t0_half;
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
    return svpwm_sector2(u_alphabeta);
}
