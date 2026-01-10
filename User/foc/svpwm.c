#include "svpwm.h"

// /**
//  * @brief  SVPWM调制函数 (min-max零序注入法)
//  * @param  u_alphabeta - αβ轴电压 (V)，为了接口兼容性
//  * @retval duty - 输出的三相占空比 (范围 0.0 ~ 0.5)
//  */
// abc_t svpwm_update(alphabeta_t u_alphabeta)
// {
//     abc_t duty;
//     float u_max, u_min, u_zero;

//     /* 反Clark变换: αβ -> abc */
//     abc_t u_abc = iclark_transform(u_alphabeta);

//     /* 归一化到 [-1, 1] */
//     u_abc.a /= (U_DC * 0.5f);
//     u_abc.b /= (U_DC * 0.5f);
//     u_abc.c /= (U_DC * 0.5f);

//     /* 找最大最小值 */
//     u_max = u_abc.a;
//     if (u_abc.b > u_max) u_max = u_abc.b;
//     if (u_abc.c > u_max) u_max = u_abc.c;

//     u_min = u_abc.a;
//     if (u_abc.b < u_min) u_min = u_abc.b;
//     if (u_abc.c < u_min) u_min = u_abc.c;

//     /* 计算零序分量 (min-max方法) */
//     u_zero = -0.5f * (u_max + u_min);

//     /* 注入零序分量，映射到占空比 [0, 1] */
//     duty.a = (u_abc.a + u_zero) * 0.5f + 0.5f;
//     duty.b = (u_abc.b + u_zero) * 0.5f + 0.5f;
//     duty.c = (u_abc.c + u_zero) * 0.5f + 0.5f;

//     /* 占空比限幅 */
//     if (duty.a > 1.0f) duty.a = 1.0f;
//     else if (duty.a < 0.0f) duty.a = 0.0f;

//     if (duty.b > 1.0f) duty.b = 1.0f;
//     else if (duty.b < 0.0f) duty.b = 0.0f;

//     if (duty.c > 1.0f) duty.c = 1.0f;
//     else if (duty.c < 0.0f) duty.c = 0.0f;

//     /* 映射到 [0, 0.5] 与硬件接口匹配 */
//     duty.a *= 0.5f;
//     duty.b *= 0.5f;
//     duty.c *= 0.5f;

//     return duty;
// }


/**
 * @brief  标准SVPWM调制函数 (七段式，中心对齐PWM)
 * @param  u_alphabeta - αβ轴电压 (V)
 * @retval duty - 输出的三相占空比 (范围 0.0 ~ 0.5)
 * @note   参考《现代永磁同步电机控制原理及MATLAB仿真》 2.4.2节
 * @attention 设置CCR寄存器要乘2，因为是中心对齐模式，详见 tim1_set_pwm_duty()
 */
abc_t svpwm_update(alphabeta_t u_alphabeta)
{
    abc_t duty;
    int32_t sector = 0;
    float Tx, Ty, Ta, Tb, Tc;
    float Tcmp1 = 0.0f, Tcmp2 = 0.0f, Tcmp3 = 0.0f;
    float f_temp;
    float v_alpha = u_alphabeta.alpha;
    float v_beta = u_alphabeta.beta;

    /* 扇区判断 */
    if (v_beta > 0.0f)
    {
        sector = 1;
    }
    if ((1.732051f * v_alpha - v_beta) / 2.0f > 0.0f)
    {
        sector += 2;
    }
    if ((-1.732051f * v_alpha - v_beta) / 2.0f > 0.0f)
    {
        sector += 4;
    }

    /* 计算矢量作用时间 Tx, Ty (归一化到 Tpwm=1.0) */
    switch (sector)
    {
    case 1:
        Tx = (-1.5f * v_alpha + 0.866025f * v_beta) / U_DC;
        Ty = (1.5f * v_alpha + 0.866025f * v_beta) / U_DC;
        break;
    case 2:
        Tx = (1.5f * v_alpha + 0.866025f * v_beta) / U_DC;
        Ty = -(1.732051f * v_beta) / U_DC;
        break;
    case 3:
        Tx = -((-1.5f * v_alpha + 0.866025f * v_beta) / U_DC);
        Ty = (1.732051f * v_beta) / U_DC;
        break;
    case 4:
        Tx = -(1.732051f * v_beta) / U_DC;
        Ty = (-1.5f * v_alpha + 0.866025f * v_beta) / U_DC;
        break;
    case 5:
        Tx = (1.732051f * v_beta) / U_DC;
        Ty = -((1.5f * v_alpha + 0.866025f * v_beta) / U_DC);
        break;
    default: /* sector 6 */
        Tx = -((1.5f * v_alpha + 0.866025f * v_beta) / U_DC);
        Ty = -((-1.5f * v_alpha + 0.866025f * v_beta) / U_DC);
        break;
    }

    /* 过调制处理：限制 Tx + Ty <= 1.0 */
    f_temp = Tx + Ty;
    if (f_temp > 1.0f)
    {
        Tx /= f_temp;
        Ty /= f_temp;
    }

    /* 计算七段式SVPWM的切换时间点 */
    Ta = (1.0f - (Tx + Ty)) / 4.0f; /* 零矢量时间/4 */
    Tb = Tx / 2.0f + Ta;
    Tc = Ty / 2.0f + Tb;

    /* 根据扇区分配三相占空比 */
    switch (sector)
    {
    case 1:
        Tcmp1 = Tb;
        Tcmp2 = Ta;
        Tcmp3 = Tc;
        break;
    case 2:
        Tcmp1 = Ta;
        Tcmp2 = Tc;
        Tcmp3 = Tb;
        break;
    case 3:
        Tcmp1 = Ta;
        Tcmp2 = Tb;
        Tcmp3 = Tc;
        break;
    case 4:
        Tcmp1 = Tc;
        Tcmp2 = Tb;
        Tcmp3 = Ta;
        break;
    case 5:
        Tcmp1 = Tc;
        Tcmp2 = Ta;
        Tcmp3 = Tb;
        break;
    case 6:
        Tcmp1 = Tb;
        Tcmp2 = Tc;
        Tcmp3 = Ta;
        break;
    default:
        Tcmp1 = 0.5f;
        Tcmp2 = 0.5f;
        Tcmp3 = 0.5f;
        break;
    }

    duty.a = Tcmp1;
    duty.b = Tcmp2;
    duty.c = Tcmp3;
    return duty;
}
