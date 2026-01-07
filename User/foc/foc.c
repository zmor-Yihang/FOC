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
    handle->motor_dir = 1;  /* 默认正向 */
}

/* 电机d轴对齐（带方向检测） */
void foc_alignment(foc_t *handle)
{
    printf("Motor Alignment Start...\n");

    float sum = 0;
    float theta_open = 0.0f;           /* 开环角度 */
    float angle_start = 0.0f;          /* 旋转前角度 */
    float angle_end = 0.0f;            /* 旋转后角度 */
    int8_t motor_dir = 1;              /* 电机方向：1=正向，-1=反向 */

    /* D轴注入1.0V，将转子拉到电气角度0度 */
    dq_t v_dq = {.d = 1.0f, .q = 0.0f};
    alphabeta_t v_alphabeta = ipark_transform(v_dq, 0.0f);
    abc_t duty = svpwm_update(v_alphabeta);
    tim1_set_pwm_duty(duty.a, duty.b, duty.c);

    HAL_Delay(500);

    /* 读取多次取平均，减少噪声 */
    sum = 0;
    for (int i = 0; i < 100; i++)
    {
        sum += as5047_get_angle_rad();
        HAL_Delay(1);
    }
    float pos_init = sum / 100.0f;  /* 第一次零点记录 */
    angle_start = pos_init;

    printf("First zero point: %.4f rad\n", pos_init);

    // 方向检测与旋转

    v_dq.d = 0.5f;
    v_dq.q = 0.0f;

    float theta_increment = 2.0f * MOTOR_POLE_PAIR * (M_PI / 180.0f);  /* 14度转弧度 ≈ 0.2443 rad */

    for (int i = 0; i < 180; i++)  /* 180次 × 10ms = 1.8s */
    {
        theta_open += theta_increment;

        v_alphabeta = ipark_transform(v_dq, theta_open);
        duty = svpwm_update(v_alphabeta);
        tim1_set_pwm_duty(duty.a, duty.b, duty.c);

        HAL_Delay(10);  /* 10ms周期 */
    }

    /* 读取旋转后的角度 */
    sum = 0;
    for (int i = 0; i < 100; i++)
    {
        sum += as5047_get_angle_rad();
        HAL_Delay(1);
    }
    angle_end = sum / 100.0f;

    /* 计算角度变化，判断方向 */
    float angle_diff = angle_end - angle_start;

    /* 处理角度翻转 */
    if (angle_diff > M_PI)
        angle_diff -= 2.0f * M_PI;
    else if (angle_diff < -M_PI)
        angle_diff += 2.0f * M_PI;

    /* 判断方向：开环正转时，如果实际角度减小，说明方向相反 */
    if (angle_diff < 0)
    {
        motor_dir = -1;
        printf("Motor direction: REVERSE\n");
    }
    else
    {
        motor_dir = 1;
        printf("Motor direction: FORWARD\n");
    }

    /* 再次将目标角度设为0，D轴电压1.0V，精确对齐 */
    v_dq.d = 1.0f;
    v_dq.q = 0.0f;
    v_alphabeta = ipark_transform(v_dq, 0.0f);
    duty = svpwm_update(v_alphabeta);
    tim1_set_pwm_duty(duty.a, duty.b, duty.c);

    HAL_Delay(500);

    /* 再次读取零点 */
    sum = 0;
    for (int i = 0; i < 100; i++)
    {
        sum += as5047_get_angle_rad();
        HAL_Delay(1);
    }
    pos_init = sum / 100.0f;  /* 更新零点 */

    printf("Final zero point: %.4f rad\n", pos_init);

    /* 关断PWM（50%占空比，安全状态） */
    tim1_set_pwm_duty(0.5f, 0.5f, 0.5f);

    /* 保存对齐结果 */
    handle->angle_offset = pos_init;
    handle->motor_dir = motor_dir;

    printf("Motor Alignment Complete! Offset=%.4f rad, Dir=%d\n", 
           handle->angle_offset, handle->motor_dir);
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
