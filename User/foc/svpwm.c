#include "svpwm.h"

// /**
//  * @brief  SVPWM调制函数 (中心对齐PWM，带零序分量注入)
//  * @param  u_abc - 输入的三相调制电压 (绝对电压值, 单位: V)
//  * @retval  duty - 输出的三相占空比 (范围 0.0 ~ 1.0)
//  */
// abc_t svpwm_update(abc_t u_abc)
// {
//     abc_t duty;
//     float u_max, u_min, u_zero;

//     u_abc.a /= (U_DC * 0.5f);
//     u_abc.b /= (U_DC * 0.5f);
//     u_abc.c /= (U_DC * 0.5f);

//     u_max = u_abc.a;
//     if (u_abc.b > u_max)
//         u_max = u_abc.b;
//     if (u_abc.c > u_max)
//         u_max = u_abc.c;

//     u_min = u_abc.a;
//     if (u_abc.b < u_min)
//         u_min = u_abc.b;
//     if (u_abc.c < u_min)
//         u_min = u_abc.c;

//     /* 计算零序分量 (min-max方法) */
//     u_zero = -0.5f * (u_max + u_min);

//     /* 注入零序分量后，将电压从 [-1, 1] 映射到占空比 [0, 1]*/
//     duty.a = (u_abc.a + u_zero) * 0.5f + 0.5f;
//     duty.b = (u_abc.b + u_zero) * 0.5f + 0.5f;
//     duty.c = (u_abc.c + u_zero) * 0.5f + 0.5f;

//     // 占空比限幅保护, 防止过调制
//     if (duty.a > 1.0f) duty.a = 1.0f;
//     else if (duty.a < 0.0f) duty.a = 0.0f;

//     if (duty.b > 1.0f) duty.b = 1.0f;
//     else if (duty.b < 0.0f) duty.b = 0.0f;

//     if (duty.c > 1.0f) duty.c = 1.0f;
//     else if (duty.c < 0.0f) duty.c = 0.0f;

//     return duty;
// }

/**
 * @brief  标准SVPWM调制函数 (七段式，中心对齐PWM)
 * @param  u_alpha - α轴电压 (V)
 * @param  u_beta  - β轴电压 (V)
 * @retval duty - 输出的三相占空比 (范围 0.0 ~ 1.0)
 */
abc_t svpwm_update(alphabeta_t u_alphabeta)
{
    abc_t duty;
    uint8_t sector;
    float x, y, z, u_alpha, u_beta;
    float time_phase_a = 0, time_phase_b = 0, time_phase_c = 0;

    // 归一化到周期T
    u_alpha = u_alphabeta.alpha * 1.732051f / U_DC; // Valpha * sqrt(3) / Udc
    u_beta = -u_alphabeta.beta / U_DC;              // -Vbeta / Udc

    // 计算中间变量x, y, z
    x = u_beta;
    y = (u_beta + u_alpha) / 2.0f;
    z = (u_beta - u_alpha) / 2.0f;

    // 判断扇区
    if (y < 0 && z < 0)
        sector = 5;
    else if (y >= 0 && z >= 0)
        sector = 2;
    else if (y < 0 && z >= 0)
        sector = (x > 0) ? 3 : 4;
    else // y >= 0 && z < 0
        sector = (x > 0) ? 1 : 6;

    // 计算各相的作用时间
    switch (sector)
    {
    case 1:
    case 4:
        time_phase_a = (1.0f + x - z) / 2.0f;
        time_phase_b = time_phase_a + z;
        time_phase_c = time_phase_b - x;
        break;

    case 2:
    case 5:
        time_phase_a = (1.0f + y - z) / 2.0f;
        time_phase_b = time_phase_a + z;
        time_phase_c = time_phase_a - y;
        break;

    case 3:
    case 6:
        time_phase_a = (1.0f - x + y) / 2.0f;
        time_phase_c = time_phase_a - y;
        time_phase_b = time_phase_c + x;
        break;

    default:
        time_phase_a = 0.5f;
        time_phase_b = 0.5f;
        time_phase_c = 0.5f;
        break;
    }

    // 防止过调制 
    duty.a = (time_phase_a > 1.0f) ? 1.0f : ((time_phase_a < 0.0f) ? 0.0f : time_phase_a);
    duty.b = (time_phase_b > 1.0f) ? 1.0f : ((time_phase_b < 0.0f) ? 0.0f : time_phase_b);
    duty.c = (time_phase_c > 1.0f) ? 1.0f : ((time_phase_c < 0.0f) ? 0.0f : time_phase_c);

    return duty;
}
