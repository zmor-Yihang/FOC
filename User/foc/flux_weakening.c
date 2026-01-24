#include "flux_weakening.h"

void flux_weak_init(flux_weak_t *flux_weak, float u_dc, float u_ref_ratio, float ki, float id_min)
{
    flux_weak->id_ref = 0.0f;
    flux_weak->u_dc = u_dc;
    flux_weak->u_ref_ratio = u_ref_ratio;
    
    /* 初始化 PID 控制器: 纯积分控制 (Kp=0) */
    /* 输出范围: [id_min, 0] */
    pid_init(&flux_weak->pid, 0.0f, ki, id_min, 0.0f);

    flux_weak->u_current_filtered = 0.0f;
    flux_weak->voltage_filter_const = 0.02f; /* 默认滤波系数，可根据需要调整 */
}

float flux_weak_calculate(flux_weak_t *flux_weak, float v_d, float v_q)
{
    /* 1. 计算当前电压模值 */
    float u_mag = sqrtf(v_d * v_d + v_q * v_q);

    /* 2. 简单的低通滤波 */
    flux_weak->u_current_filtered = flux_weak->u_current_filtered * (1.0f - flux_weak->voltage_filter_const) + u_mag * flux_weak->voltage_filter_const;

    /* 3. 计算目标电压参考值 */
    float u_ref = flux_weak->u_dc * flux_weak->u_ref_ratio;

    /* 4. 使用 PID 控制器计算弱磁电流 */
    /* Setpoint = u_ref, Feedback = u_current_filtered */
    /* 当 u_current > u_ref (反馈>目标) 时，误差 < 0，PID 输出负值 (弱磁电流) */
    flux_weak->id_ref = pid_calculate(&flux_weak->pid, u_ref, flux_weak->u_current_filtered);

    return flux_weak->id_ref;
}
