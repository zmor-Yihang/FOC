#include "test_speed_closed_loop.h"

/* 静态变量定义 */
static pid_controller_t pid_speed;             /* 速度环PID */
static pid_controller_t pid_id;                /* D轴电流环PID */
static pid_controller_t pid_iq;                /* Q轴电流环PID */
static foc_t foc_handle;                       /* FOC控制句柄 */
static volatile uint8_t speed_loop_enable = 0; /* 速度环使能标志 */
static volatile float target_speed_ramp = 0.0f;         /* 目标速度斜坡值 */
static float target_speed_final = 0.0f;        /* 最终目标速度 */
static volatile uint8_t stop_request = 0;      /* 停机请求标志 */
static uint8_t step_index = 0;                 /* 当前阶梯索引 (0-6) */
static uint32_t step_counter = 0;              /* 阶梯保持计数器 */
static uint32_t step_hold_time = 2000;         /* 每个阶梯保持时间（中断次数） */
static float step_speed_start = 0.0f;          /* 当前阶梯起始速度 */
static float step_speed_target = 0.0f;         /* 当前阶梯目标速度 */
static float ramp_increment = 0.5f;            /* 斜坡增量 (RPM/中断) */
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
             0.00002f, /* Ki */
             0.0f,     /* Kd */
             2.0f,     /* 输出上限(目标Iq) */
             -2.0f     /* 输出下限 */
    );

    /* 电流环输出限幅不超过Udc/√3，避免过调制 */
    float v_limit = U_DC * 0.557f; /* 约6.75V */
    pid_init(&pid_id, 0.017f, 0.00035f, 0.0f, v_limit / 2, -v_limit / 2);
    pid_init(&pid_iq, 0.017f, 0.00035f, 0.0f, v_limit / 2, -v_limit / 2);

    /* 初始化FOC句柄 - 传入速度环PID控制器 */
    foc_init(&foc_handle, &pid_id, &pid_iq, &pid_speed);

    /* 电机零点对齐 */
    foc_alignment(&foc_handle);

    /* 软启动：初始目标速度为0（单位：RPM） */
    target_speed_final = 6000.0f;
    target_speed_ramp = 0.0f;
    step_index = 0;
    step_counter = 0;
    step_speed_start = 0.0f;
    step_speed_target = 0.0f;
    foc_set_target_id(&foc_handle, 0.0f);
    foc_set_target_iq(&foc_handle, 0.0f);
    foc_set_target_speed(&foc_handle, 0.0f);

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
            /* 设置停机请求标志 */
            stop_request = 1;

            /* 设置目标速度为0，让速度环自然减速 */
            target_speed_final = 0.0f;
            foc_set_target_speed(&foc_handle, 0.0f);

            printf("Stop request sent, decelerating to 0 RPM...\r\n");
        }

        /* 检查是否停机完成 */
        if (stop_request && fabsf(actual_speed) < 10.0f)
        {
            /* 速度降到接近0，关闭速度环 */
            speed_loop_enable = 0;

            /* 停止PWM输出 */
            tim1_set_pwm_duty(0, 0, 0);

            /* 复位PID */
            pid_reset(&pid_speed);
            pid_reset(&pid_id);
            pid_reset(&pid_iq);

            printf("Motor stopped!\r\n");
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

    /* 阶梯启动：6个阶梯，每个阶梯内部使用斜坡加速 */
    if (target_speed_ramp < target_speed_final)
    {
        if (step_index == 0)
        {
            step_speed_start = 0.0f;
            step_speed_target = 666.67f;
            step_index = 1;
        }

        if (target_speed_ramp < step_speed_target)
        {
            /* 当前阶梯内部斜坡加速 */
            target_speed_ramp += ramp_increment;
            if (target_speed_ramp > step_speed_target)
            {
                target_speed_ramp = step_speed_target;
            }
        }
        else
        {
            /* 达到当前阶梯目标速度，开始保持计时 */
            step_counter++;
            if (step_counter >= step_hold_time)
            {
                step_counter = 0;
                if (step_index < 6)
                {
                    step_index++;
                    step_speed_start = step_speed_target;
                    step_speed_target = (float)step_index * 666.67f;
                    if (step_speed_target > target_speed_final)
                    {
                        step_speed_target = target_speed_final;
                    }
                }
            }
        }
        foc_handle.target_speed = target_speed_ramp;
    }
    else if (target_speed_ramp > target_speed_final)
    {
        /* 减速阶梯：按阶梯降低目标速度 */
        step_counter++;
        if (step_counter >= step_hold_time)
        {
            step_counter = 0;
            if (step_index > 0)
            {
                step_index--;
            }
        }
        /* 阶梯速度：每个阶梯减少666.67 RPM */
        target_speed_ramp = (float)step_index * 666.67f;
        if (target_speed_ramp < target_speed_final)
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

    /* Clark变换: abc -> αβ */
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    /* Park变换: αβ -> dq */
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    /* 保存到可打印变量（ISR 内写入，主循环读取打印） */
    current_d = i_dq.d;
    current_q = i_dq.q;

    /* 执行速度闭环控制：外环速度PID + 内环电流PID */
    foc_speed_closed_loop(&foc_handle, angle_el, i_dq, actual_speed_rpm);
}