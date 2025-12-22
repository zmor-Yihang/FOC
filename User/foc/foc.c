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
}

void foc_set_target(foc_t *handle, float target_Id, float target_Iq, float target_speed)
{
    handle->target_Id = target_Id;
    handle->target_Iq = target_Iq;
    handle->target_speed = target_speed;
}

/* 电机零点对齐 */
void foc_alignment()
{
    printf("Motor Alignment Start...\n");

    /* 对齐参数 */
    float align_voltage = 0.1f; /* d轴加的电压 10% Udc */
    float align_angle = 0.0f;   /* 目标电角度0 */
    int align_steps = 1000;     /* 对齐步数 */

    for (int i = 0; i < align_steps; i++)
    {
        /* 电压从0渐变到目标值 */
        float ramp = (float)i / align_steps;

        dq_t v_dq = {
            .d = align_voltage * ramp,
            .q = 0,
        };
        alphabeta_t v_alphabeta = ipark_transform(v_dq, align_angle);
        abc_t v_abc = iclark_transform(v_alphabeta);
        abc_t duty = svpwm_update(v_abc);
        tim1_set_pwm_duty(duty.a, duty.b, duty.c);

        HAL_Delay(2);
    }
    tim1_set_pwm_duty(0, 0, 0);
    printf("Motor ALigment Stop!\n");
}

/* 开环运行 */
void foc_open_loop(dq_t u_dq, float angle_rad_el)
{
    alphabeta_t alpha_beta = ipark_transform(u_dq, angle_rad_el);
    abc_t u_abc = iclark_transform(alpha_beta);
    abc_t duty_abc = svpwm_update(u_abc);
    tim1_set_pwm_duty(duty_abc.a, duty_abc.b, duty_abc.c);
}

void foc_current_closed_loop(foc_t *handle, dq_t i_dq_feedback, float angle_el)
{
    handle->v_d_out = pid_calculate(handle->pid_id, handle->target_Id, i_dq_feedback.d);
    handle->v_q_out = pid_calculate(handle->pid_iq, handle->target_Iq, i_dq_feedback.q);

    alphabeta_t v_alphabeta = ipark_transform((dq_t){.d = handle->v_d_out, .q = handle->v_q_out}, angle_el);

    abc_t v_abc = iclark_transform(v_alphabeta);

    handle->duty_cycle = svpwm_update(v_abc);

    tim1_set_pwm_duty(handle->duty_cycle.a, handle->duty_cycle.b, handle->duty_cycle.c);
}

void foc_loop(foc_t *handle, float angle_el, abc_t *i_abc, uint16_t speed_rpm)
{
    
}