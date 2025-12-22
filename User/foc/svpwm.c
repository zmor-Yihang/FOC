#include "svpwm.h"

/**
 * @brief  SVPWM调制函数 (中心对齐PWM，带零序分量注入)
 * @param  u_abc - 输入的三相调制电压 (绝对电压值, 单位: V)
 * @retval  duty - 输出的三相占空比 (范围 0.0 ~ 1.0)
 */
abc_t svpwm_update(abc_t u_abc)
{
    abc_t duty;
    float u_max, u_min, u_zero;

    u_abc.a /= U_DC;
    u_abc.b /= U_DC;
    u_abc.c /= U_DC;

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

    /* 注入零序分量后，将电压从 [-1, 1] 映射到占空比 [0, 1]*/
    duty.a = (u_abc.a + u_zero) * 0.5f + 0.5f;
    duty.b = (u_abc.b + u_zero) * 0.5f + 0.5f;
    duty.c = (u_abc.c + u_zero) * 0.5f + 0.5f;

    // 占空比限幅保护, 防止过调制
    if (duty.a > 1.0f) duty.a = 1.0f;
    else if (duty.a < 0.0f) duty.a = 0.0f;

    if (duty.b > 1.0f) duty.b = 1.0f;
    else if (duty.b < 0.0f) duty.b = 0.0f;

    if (duty.c > 1.0f) duty.c = 1.0f;
    else if (duty.c < 0.0f) duty.c = 0.0f;

    return duty;
}


