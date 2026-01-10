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
    handle->open_loop_angle_el = 0.0f;
}

void foc_alignment(foc_t *handle)
{
    /* 施加d轴电压，让转子对齐到电角度0位置 */
    dq_t u_dq = {.d = 1.0f, .q = 0.0f};
    
    /* 开环输出，固定电角度为0 */
    alphabeta_t alpha_beta = ipark_transform(u_dq, 0);
    abc_t duty_abc = svpwm_update(alpha_beta);
    tim1_set_pwm_duty(duty_abc.a, duty_abc.b, duty_abc.c);
    
    /* 等待转子稳定 */
    HAL_Delay(100);
    
    /* 读取当前电角度作为零点偏移 */
    handle->angle_offset = as5047_get_angle_rad();
    
    /* 关闭PWM输出 */
    tim1_set_pwm_duty(0.5f, 0.5f, 0.5f);
}

/**
 * @brief 开环速度运行 - 在定时中断中调用 (10kHz)
 * @param handle    FOC 控制句柄
 * @param speed_rpm 目标转速 (RPM)，是旋转磁场速度，不能太大，否则电机会失步
 * @param voltage_q Q轴电压幅值 (V)
 */
void foc_open_loop_run(foc_t *handle, float speed_rpm, float voltage_q)
{
    /* 计算每次中断的角度增量
     * delta_angle = 2π × 极对数 × (转速RPM / 60) × 采样周期
     * 采样周期 = 1/10000 = 0.0001s
     */
    float delta_angle = 2.0f * M_PI * AS5047_MOTOR_POLE_PAIR * 
                        (speed_rpm / 60.0f) * 0.0001f;
    
    /* 累加电角度 */
    handle->open_loop_angle_el += delta_angle;
    
    /* 输出电压矢量：d轴为0，q轴为设定电压 */
    dq_t u_dq = {.d = 0.0f, .q = voltage_q};
    
    /* 执行开环输出 */
    alphabeta_t alpha_beta = ipark_transform(u_dq, handle->open_loop_angle_el);
    abc_t duty_abc = svpwm_update(alpha_beta);
    tim1_set_pwm_duty(duty_abc.a, duty_abc.b, duty_abc.c);
}

/**
 * @brief 电流闭环运行
 * @param handle    FOC 控制句柄
 * @param i_dq      dq 轴电流反馈
 * @param angle_el  电角度 (rad)
 */
void foc_current_closed_loop_run(foc_t *handle, dq_t i_dq, float angle_el)
{
    /* 电流环 PID */
    handle->v_d_out = pid_calculate(handle->pid_id, handle->target_id, i_dq.d);
    handle->v_q_out = pid_calculate(handle->pid_iq, handle->target_iq, i_dq.q);

    /* 逆 Park 变换 */
    alphabeta_t v_alphabeta = ipark_transform((dq_t){.d = handle->v_d_out, .q = handle->v_q_out}, angle_el);

    /* SVPWM 输出 */
    handle->duty_cycle = svpwm_update(v_alphabeta);
    tim1_set_pwm_duty(handle->duty_cycle.a, handle->duty_cycle.b, handle->duty_cycle.c);
}

/**
 * @brief 速度闭环运行
 * @param handle    FOC 控制句柄
 * @param i_dq      dq 轴电流反馈
 * @param angle_el  电角度 (rad)
 * @param speed_rpm 速度反馈 (RPM)
 */
void foc_speed_closed_loop_run(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm)
{
    /* 速度环 → 输出目标 Iq */
    handle->target_iq = pid_calculate(handle->pid_speed, handle->target_speed, speed_rpm);

    /* Id 目标设为 0 */
    handle->target_id = 0.0f;

    /* 复用电流闭环 */
    foc_current_closed_loop_run(handle, i_dq, angle_el);
}

/**
 * @brief 停止闭环运行，复位 PID 状态
 */
void foc_closed_loop_stop(foc_t *handle)
{
    /* 复位所有 PID 控制器，清除积分项 */
    pid_reset(handle->pid_id);
    pid_reset(handle->pid_iq);
    pid_reset(handle->pid_speed);

    /* 清除目标值 */
    handle->target_id = 0.0f;
    handle->target_iq = 0.0f;
    handle->target_speed = 0.0f;

    /* 清除输出 */
    handle->v_d_out = 0.0f;
    handle->v_q_out = 0.0f;

    /* 输出50%占空比，电机停止 */
    tim1_set_pwm_duty(0.5f, 0.5f, 0.5f);
}