#include "test_speed_closed_loop.h"

/* 静态变量定义 */
static pid_controller_t pid_speed;             /* 速度环PID */
static pid_controller_t pid_id;                /* D轴电流环PID */
static pid_controller_t pid_iq;                /* Q轴电流环PID */
static foc_t foc_handle;                       /* FOC控制句柄 */
static volatile uint8_t speed_loop_enable = 0; /* 速度环使能标志 */
static float target_speed_ramp = 0.0f;         /* 目标速度斜坡值 */
static float target_speed_final = 0.0f;        /* 最终目标速度 */
/* 供打印使用：在 ISR 中写入，在主循环打印，避免在 ISR 中阻塞 */
static volatile float current_speed = 0.0f;
static volatile float actual_speed = 0.0f;
static volatile float current_d = 0.0f;
static volatile float current_q = 0.0f;

/* 速度闭环测试函数 */
void test_speed_closed_loop(void)
{
    /* 初始化PID控制器 - 速度环参数 */
    /* 速度环输出限幅作为Q轴电流目标值 */
    pid_init(&pid_speed,
             0.05f,    /* Kp - 速度环较慢 */
             0.00001f, /* Ki */
             0.0f,     /* Kd */
             2.0f,     /* 输出上限(目标Iq) */
             -2.0f     /* 输出下限 */
    );

    /* 电流环输出限幅不超过Udc/√3，避免过调制 */
    float v_limit = U_DC * 0.557f; /* 约6.75V */
    pid_init(&pid_id, 0.017f, 0.000035f, 0.0f, v_limit, -v_limit);
    pid_init(&pid_iq, 0.017f, 0.000035f, 0.0f, v_limit, -v_limit);

    /* 初始化FOC句柄 - 传入速度环PID控制器 */
    foc_init(&foc_handle, &pid_id, &pid_iq, &pid_speed);

    /* 电机零点对齐 */
    foc_alignment(&foc_handle);

    /* 软启动：初始目标速度为0（单位：RPM） */
    target_speed_final = -4000.0f;
    target_speed_ramp = 0.0f;
    foc_set_target(&foc_handle, 0.0f, 0.0f, 0.0f);

    printf("Speed Closed Loop Test Start!\r\n");
    printf("Target Speed: %.2f RPM\r\n", target_speed_final);

    /* 使能速度环 */
    speed_loop_enable = 1;

    /* 主循环：等待按键退出 */
    while (1)
    {
        /* 延时，避免CPU占用过高 */
        delay_us(100);

        /* 每 100ms 打印一次当前速度和电流 */
        printf_period(100, "Speed ID IQ: %.3f, %.3f, %.3f\n",
                      actual_speed, current_d, current_q);

        /* 按键检测，按下退出 */
        if (key_scan() == 1)
        {
            /* 关闭速度环 */
            speed_loop_enable = 0;

            /* 停止PWM输出 */
            tim1_set_pwm_duty(0, 0, 0);

            /* 复位PID */
            pid_reset(&pid_speed);
            pid_reset(&pid_id);
            pid_reset(&pid_iq);

            break;
        }
        /* no-op */
    }

    printf("Speed Closed Loop Test Stop!\r\n");
}

/* ADC注入组中断回调中调用的速度闭环处理函数 */
void speed_closed_loop_handler(void)
{
    if (!speed_loop_enable)
    {
        return;
    }

    /* 软启动斜坡：逐渐增加目标速度 */
    if (target_speed_ramp < target_speed_final)
    {
        /* 斜坡增量：以 RPM 为单位 */
        target_speed_ramp += 0.1f; /* 根据中断频率调整为合适的 RPM 步进 */
        if (target_speed_ramp > target_speed_final)
        {
            target_speed_ramp = target_speed_final;
        }
        /* 直接把 RPM 目标写入 foc_handle */
        foc_handle.target_speed = target_speed_ramp;
    }

    /* 更新速度计算 */
    as5047_update_speed();

    /* 获取实际转速 */
    float actual_speed_rpm = as5047_get_speed_rpm();
    
    /* 以 RPM 保存用于显示 */
    actual_speed = actual_speed_rpm;

    /* 获取ADC注入组转换值 */
    adc_values_t adc_values;
    adc1_get_injected_values(&adc_values);

    /* 获取电角度 */
    float angle_mech = as5047_get_angle_rad();                                 /* 机械角度 (rad) */
    float angle_el = (angle_mech - foc_handle.angle_offset) * MOTOR_POLE_PAIR; /* 电角度 (rad) */

    /* 三相电流 */
    abc_t i_abc = {
        .a = adc_values.ia,
        .b = adc_values.ib,
        .c = adc_values.ic,
    };

    /* 执行速度闭环控制：外环速度PID + 内环电流PID */
    foc_speed_closed_loop(&foc_handle, angle_el, &i_abc, (uint16_t)actual_speed_rpm);

    /* 重新计算dq电流用于显示 */
    alphabeta_t i_alphabeta = clark_transform(i_abc);
    dq_t i_dq = park_transform(i_alphabeta, angle_el);
    current_d = i_dq.d;
    current_q = i_dq.q;
}