#include "foc.h"

void foc_init(foc_t* handle, pid_controller_t *pid_id, pid_controller_t *pid_iq, pid_controller_t *pid_speed)
{
    handle->target_speed = 0;
    handle->target_id = 0;
    handle->target_iq = 0;

    handle->i_abc.a = 0.0f;
    handle->i_abc.b = 0.0f;
    handle->i_abc.c = 0.0f;

    handle->v_d_out = 0.0f;
    handle->v_q_out = 0.0f;
    handle->i_q_out = 0.0f;

    handle->pid_id = pid_id;
    handle->pid_iq = pid_iq;
    handle->pid_speed = pid_speed;

    handle->duty_cycle.a = 0.0f;
    handle->duty_cycle.b = 0.0f;
    handle->duty_cycle.c = 0.0f;

    handle->angle_offset = 0.0f;
}

void foc_alignment(foc_t *handle)
{
    /* 施加d轴电压，让转子对齐到电角度0位置 */
    dq_t u_dq = {.d = 1.0f, .q = 0.0f};
    
    /* 开环输出，固定电角度为0 */
    foc_open_loop(u_dq, 0);
    
    /* 等待转子稳定 */
    HAL_Delay(100);
    
    /* 读取当前编码器机械角度作为零点偏移 */
    float mech_angle = as5047_get_angle_rad();
    
    /* 计算电角度偏移 = 机械角度 × 极对数 */
    handle->angle_offset = mech_angle * MOTOR_POLE_PAIR;
    
    /* 归一化到 [0, 2π) */
    while (handle->angle_offset >= 2.0f * M_PI)
    {
        handle->angle_offset -= 2.0f * M_PI;
    }
    while (handle->angle_offset < 0.0f)
    {
        handle->angle_offset += 2.0f * M_PI;
    }
    
    /* 关闭PWM输出 */
    tim1_set_pwm_duty(0.5f, 0.5f, 0.5f);
}

/* 开环运行 */
void foc_open_loop(dq_t u_dq, float angle_rad_el)
{
    alphabeta_t alpha_beta = ipark_transform(u_dq, angle_rad_el);
    abc_t duty_abc = svpwm_update(alpha_beta);
    tim1_set_pwm_duty(duty_abc.a, duty_abc.b, duty_abc.c);
}

/* 电流闭环运行 */
void foc_current_closed_loop(foc_t *handle, dq_t i_dq_feedback, float angle_el)
{
    handle->v_d_out = pid_calculate(handle->pid_id, handle->target_id, i_dq_feedback.d);
    handle->v_q_out = pid_calculate(handle->pid_iq, handle->target_iq, i_dq_feedback.q);

    alphabeta_t v_alphabeta = ipark_transform((dq_t){.d = handle->v_d_out, .q = handle->v_q_out}, angle_el);

    handle->duty_cycle = svpwm_update(v_alphabeta);

    tim1_set_pwm_duty(handle->duty_cycle.a, handle->duty_cycle.b, handle->duty_cycle.c);
}

/* 速度闭环 */
void foc_speed_closed_loop(foc_t *handle, float angle_el, dq_t i_dq, float speed_rpm)
{
    // 速度环 → 输出目标 Iq
    handle->target_iq = pid_calculate(handle->pid_speed, handle->target_speed, speed_rpm);

    // Id 目标设为0
    handle->target_id = 0.0f;

    // 电流环
    handle->v_d_out = pid_calculate(handle->pid_id, handle->target_id, i_dq.d);
    handle->v_q_out = pid_calculate(handle->pid_iq, handle->target_iq, i_dq.q);

    // 逆 Park
    alphabeta_t v_alphabeta = ipark_transform((dq_t){.d = handle->v_d_out, .q = handle->v_q_out}, angle_el);

    handle->duty_cycle = svpwm_update(v_alphabeta);
    tim1_set_pwm_duty(handle->duty_cycle.a, handle->duty_cycle.b, handle->duty_cycle.c);
}
