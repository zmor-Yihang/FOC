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

/* 开环运行 - 基础版本 */
void foc_open_loop(dq_t u_dq, float angle_rad_el)
{
    alphabeta_t alpha_beta = ipark_transform(u_dq, angle_rad_el);
    abc_t duty_abc = svpwm_update(alpha_beta);
    tim1_set_pwm_duty(duty_abc.a, duty_abc.b, duty_abc.c);
}

/* 开环速度运行状态 */
static struct {
    float angle_el;         /* 当前电角度 (rad) */
    float target_speed_rpm; /* 目标转速 (RPM) */
    float voltage_q;        /* Q轴电压 (V) */
} open_loop_state = {0.0f, 0.0f, 0.0f};

/**
 * @brief 设置开环运行参数
 * @param speed_rpm 目标转速 (RPM)，正值正转，负值反转
 * @param voltage_q Q轴电压幅值 (V)
 */
void foc_open_loop_set(float speed_rpm, float voltage_q)
{
    open_loop_state.target_speed_rpm = speed_rpm;
    open_loop_state.voltage_q = voltage_q;
}

/**
 * @brief 开环速度运行 - 在定时中断中调用 (10kHz)
 * @note  通过递增电角度实现开环转速控制
 */
void foc_open_loop_run(void)
{
    /* 计算每次中断的角度增量
     * delta_angle = 2π × 极对数 × (转速RPM / 60) × 采样周期
     * 采样周期 = 1/10000 = 0.0001s
     */
    float delta_angle = 2.0f * M_PI * MOTOR_POLE_PAIR * 
                        (open_loop_state.target_speed_rpm / 60.0f) * 0.0001f;
    
    /* 累加电角度 */
    open_loop_state.angle_el += delta_angle;
    
    /* 归一化到 [0, 2π) */
    if (open_loop_state.angle_el >= 2.0f * M_PI)
    {
        open_loop_state.angle_el -= 2.0f * M_PI;
    }
    else if (open_loop_state.angle_el < 0.0f)
    {
        open_loop_state.angle_el += 2.0f * M_PI;
    }
    
    /* 输出电压矢量：d轴为0，q轴为设定电压 */
    dq_t u_dq = {.d = 0.0f, .q = open_loop_state.voltage_q};
    
    /* 执行开环输出 */
    foc_open_loop(u_dq, open_loop_state.angle_el);
}

/**
 * @brief 电流闭环运行
 * @param handle    FOC 控制句柄
 * @param i_dq      dq 轴电流反馈
 * @param angle_el  电角度 (rad)
 */
void foc_current_closed_loop(foc_t *handle, dq_t i_dq, float angle_el)
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
void foc_speed_closed_loop(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm)
{
    /* 速度环 → 输出目标 Iq */
    handle->target_iq = pid_calculate(handle->pid_speed, handle->target_speed, speed_rpm);

    /* Id 目标设为 0 */
    handle->target_id = 0.0f;

    /* 复用电流闭环 */
    foc_current_closed_loop(handle, i_dq, angle_el);
}

/**
 * @brief 停止开环运行
 */
void foc_open_loop_stop(void)
{
    open_loop_state.target_speed_rpm = 0.0f;
    open_loop_state.voltage_q = 0.0f;
    open_loop_state.angle_el = 0.0f;
    tim1_set_pwm_duty(0.5f, 0.5f, 0.5f);
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