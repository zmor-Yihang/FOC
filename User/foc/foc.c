#include "foc.h"

void foc_init(foc_t *handle, pid_controller_t *pid_id, pid_controller_t *pid_iq, pid_controller_t *pid_speed)
{
    handle->target_speed = 0;
    handle->target_Id = 0;
    handle->target_Iq = 0;

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

void foc_set_target(foc_t *handle, float target_Id, float target_Iq, float target_speed)
{
    handle->target_Id = target_Id;
    handle->target_Iq = target_Iq;
    handle->target_speed = target_speed;
}

/* 电机d轴对齐 */
void foc_alignment(foc_t *handle)
{
    printf("Motor Alignment Start...\n");

    dq_t v_dq = {
        .d = 2.0f,
        .q = 0.0f,
    };

    alphabeta_t v_alphabeta = ipark_transform(v_dq, 0.0f);
    abc_t duty = svpwm_update(v_alphabeta);
    tim1_set_pwm_duty(duty.a, duty.b, duty.c);

    HAL_Delay(2000);

    tim1_set_pwm_duty(0, 0, 0);
    float angle_offset = as5047_get_angle_rad();
    handle->angle_offset = angle_offset;

    printf("Motor Alignment Stop!\n");
    printf("Angle Offset: %.4f rad (%.2f deg)\n", handle->angle_offset, handle->angle_offset * 180.0f / M_PI);
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
    handle->v_d_out = pid_calculate(handle->pid_id, handle->target_Id, i_dq_feedback.d);
    handle->v_q_out = pid_calculate(handle->pid_iq, handle->target_Iq, i_dq_feedback.q);

    alphabeta_t v_alphabeta = ipark_transform((dq_t){.d = handle->v_d_out, .q = handle->v_q_out}, angle_el);

    handle->duty_cycle = svpwm_update(v_alphabeta);

    tim1_set_pwm_duty(handle->duty_cycle.a, handle->duty_cycle.b, handle->duty_cycle.c);
}

/* 速度闭环 */
void foc_speed_closed_loop(foc_t *handle, float angle_el, abc_t *i_abc, uint16_t speed_rpm)
{
    // 1. Clarke + Park 变换，得到 dq 电流
    alphabeta_t i_alphabeta = clark_transform(*i_abc);
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    // 2. 速度环 → 输出目标 Iq
    handle->target_Iq = pid_calculate(handle->pid_speed, handle->target_speed, (float)speed_rpm);

    // 3. Id 目标一般为 0
    handle->target_Id = 0.0f;

    // 4. 电流环
    handle->v_d_out = pid_calculate(handle->pid_id, handle->target_Id, i_dq.d);
    handle->v_q_out = pid_calculate(handle->pid_iq, handle->target_Iq, i_dq.q);

    // 5. 逆 Park + SVPWM
    alphabeta_t v_alphabeta = ipark_transform(
        (dq_t){.d = handle->v_d_out, .q = handle->v_q_out},
        angle_el);

    handle->duty_cycle = svpwm_update(v_alphabeta);
    tim1_set_pwm_duty(handle->duty_cycle.a, handle->duty_cycle.b, handle->duty_cycle.c);
}