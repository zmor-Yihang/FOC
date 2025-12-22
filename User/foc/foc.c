#include "foc.h"

void foc_init(foc_t *handle, pid_controller_t *pid_id, pid_controller_t *pid_iq, pid_controller_t *pid_speed)
{
    handle->pole_pairs = MOTOR_POLES; // 设置电机极对数
    handle->target_speed = 0;
    handle->target_Id = 0;
    handle->target_Iq = 0;

    /* 初始化电流采样值 */
    handle->i_abc.a = 0.0f;
    handle->i_abc.b = 0.0f;
    handle->i_abc.c = 0.0f;

    /* 初始化Clark变换后的电流值 */
    handle->i_alphabeta.alpha = 0.0f;
    handle->i_alphabeta.beta = 0.0f;

    /* 初始化Park变换后的电流值 */
    handle->i_dq.d = 0.0f;
    handle->i_dq.q = 0.0f;

    /* 初始化电压控制量 */
    handle->v_d = 0.0f;
    handle->v_q = 0.0f;

    /* 初始化Park变换后的电压值 */
    handle->v_alpha = 0.0f;
    handle->v_beta = 0.0f;

    /* 初始化零点偏移 */
    handle->zero_offset = 0.0f;

    /* 初始化母线电压为默认值 */
    handle->vbus = 13.15f; // 默认13.15V

    /* 初始化PID控制器指针 */
    handle->pid_id = pid_id;
    handle->pid_iq = pid_iq;
    handle->pid_speed = pid_speed;

    /* 初始化占空比为0 */
    handle->duty_cycle.a = 0.0f;
    handle->duty_cycle.b = 0.0f;
    handle->duty_cycle.c = 0.0f;
}

/* 开环运行 */
void foc_open_loop(dq_t u_dq, float angle_rad_el)
{
    alphabeta_t alpha_beta = ipark_transform(u_dq, angle_rad_el);
    abc_t u_abc = iclark_transform(alpha_beta);
    abc_t duty_abc = svpwm_update(u_abc);
    tim1_set_pwm_duty(duty_abc.a, duty_abc.b, duty_abc.c);
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

void foc_set_target_speed(foc_t *handle, float speed_rpm)
{
    handle->target_speed = speed_rpm;
}

void foc_set_target_currents(foc_t *handle, float Id, float Iq)
{
    handle->target_Id = Id;
    handle->target_Iq = Iq;
}

void foc_loop(foc_t *handle, float angle_el, abc_t *i_abc, uint16_t speed_rpm)
{
}
