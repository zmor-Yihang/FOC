#include "svpwm.h"

// /**
//  * @brief  标准七段式SVPWM (扇区法)
//  * @param  u_alphabeta - αβ轴电压 (V)
//  * @retval duty - 输出的三相占空比 (范围 0.0 ~ 0.5)
//  * @note   占空比围绕0.25中心分布，与min-max注入法输出一致
//  */
// abc_t svpwm_update(alphabeta_t u_alphabeta)
// {
//     abc_t duty;
//     int sector;
//     float u1, u2, u3;
//     float t1, t2, t0;
//     float ta, tb, tc;

//     float v_alpha = u_alphabeta.alpha;
//     float v_beta = u_alphabeta.beta;

//     /* 计算参考电压矢量在三个轴上的投影 */
//     u1 = v_beta;
//     u2 = (1.732051f * v_alpha - v_beta) * 0.5f;
//     u3 = (-1.732051f * v_alpha - v_beta) * 0.5f;

//     /* 扇区判断 (N = 4*sign(u3) + 2*sign(u2) + sign(u1)) */
//     sector = (u1 > 0) + ((u2 > 0) << 1) + ((u3 > 0) << 2);

//     /* 根据扇区计算矢量作用时间 */
//     switch (sector)
//     {
//     case 3: /* 扇区1 */
//         t1 = u2;
//         t2 = u1;
//         break;
//     case 1: /* 扇区2 */
//         t1 = -u3;
//         t2 = -u2;
//         break;
//     case 5: /* 扇区3 */
//         t1 = u1;
//         t2 = u3;
//         break;
//     case 4: /* 扇区4 */
//         t1 = -u2;
//         t2 = -u1;
//         break;
//     case 6: /* 扇区5 */
//         t1 = u3;
//         t2 = u2;
//         break;
//     case 2: /* 扇区6 */
//         t1 = -u1;
//         t2 = -u3;
//         break;
//     default:
//         t1 = 0;
//         t2 = 0;
//         break;
//     }

//     /* 归一化时间 */
//     t1 = t1 * 1.732051f / U_DC;
//     t2 = t2 * 1.732051f / U_DC;

//     /* 过调制处理 */
//     if (t1 + t2 > 1.0f)
//     {
//         float k = 1.0f / (t1 + t2);
//         t1 *= k;
//         t2 *= k;
//     }

//     t0 = 1.0f - t1 - t2;

//     /* 计算三相占空比 (中心对称分布，与min-max注入法一致) */
//     switch (sector)
//     {
//     case 3: /* 扇区1 */
//         ta = t1 + t2 + t0 * 0.5f;
//         tb = t2 + t0 * 0.5f;
//         tc = t0 * 0.5f;
//         break;
//     case 1: /* 扇区2 */
//         ta = t1 + t0 * 0.5f;
//         tb = t1 + t2 + t0 * 0.5f;
//         tc = t0 * 0.5f;
//         break;
//     case 5: /* 扇区3 */
//         ta = t0 * 0.5f;
//         tb = t1 + t2 + t0 * 0.5f;
//         tc = t2 + t0 * 0.5f;
//         break;
//     case 4: /* 扇区4 */
//         ta = t0 * 0.5f;
//         tb = t1 + t0 * 0.5f;
//         tc = t1 + t2 + t0 * 0.5f;
//         break;
//     case 6: /* 扇区5 */
//         ta = t2 + t0 * 0.5f;
//         tb = t0 * 0.5f;
//         tc = t1 + t2 + t0 * 0.5f;
//         break;
//     case 2: /* 扇区6 */
//         ta = t1 + t2 + t0 * 0.5f;
//         tb = t0 * 0.5f;
//         tc = t1 + t0 * 0.5f;
//         break;
//     default:
//         ta = tb = tc = 0.5f;
//         break;
//     }

//     /* 映射到 [0, 0.5] */
//     duty.a = ta * 0.5f;
//     duty.b = tb * 0.5f;
//     duty.c = tc * 0.5f;

//     return duty;
// }


/**
 * @brief  SVPWM调制函数 (min-max零序注入法)
 * @param  u_alphabeta - αβ轴电压 (V)，为了接口兼容性
 * @retval duty - 输出的三相占空比 (范围 0.0 ~ 0.5)
 */
abc_t svpwm_update(alphabeta_t u_alphabeta)
{
    abc_t duty;
    float u_max, u_min, u_zero;

    /* 反Clark变换: αβ -> abc */
    abc_t u_abc = iclark_transform(u_alphabeta);

    /* 归一化到 [-1, 1] */
    u_abc.a /= (U_DC * 0.5f);
    u_abc.b /= (U_DC * 0.5f);
    u_abc.c /= (U_DC * 0.5f);

    /* 找最大最小值 */
    u_max = u_abc.a;
    if (u_abc.b > u_max)
        u_max = u_abc.b;
    if (u_abc.c > u_max)
        u_max = u_abc.c;

    u_min = u_abc.a;
    if (u_abc.b < u_min)
        u_min = u_abc.b;
    if (u_abc.c < u_min)
        u_min = u_abc.c;

    /* 计算零序分量 (min-max方法) */
    u_zero = -0.5f * (u_max + u_min);

    /* 注入零序分量，映射到占空比 [0, 1] */
    duty.a = (u_abc.a + u_zero) * 0.5f + 0.5f;
    duty.b = (u_abc.b + u_zero) * 0.5f + 0.5f;
    duty.c = (u_abc.c + u_zero) * 0.5f + 0.5f;

    /* 占空比限幅 */
    if (duty.a > 1.0f)
        duty.a = 1.0f;
    else if (duty.a < 0.0f)
        duty.a = 0.0f;

    if (duty.b > 1.0f)
        duty.b = 1.0f;
    else if (duty.b < 0.0f)
        duty.b = 0.0f;

    if (duty.c > 1.0f)
        duty.c = 1.0f;
    else if (duty.c < 0.0f)
        duty.c = 0.0f;

    /* 映射到 [0, 0.5] 与硬件接口匹配 */
    duty.a *= 0.5f;
    duty.b *= 0.5f;
    duty.c *= 0.5f;

    return duty;
}