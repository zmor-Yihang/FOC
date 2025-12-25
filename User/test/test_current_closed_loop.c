#include "test_current_closed_loop.h"

/* 静态变量定义 */
static pid_controller_t pid_id;  /* D轴电流环PID */
static pid_controller_t pid_iq;  /* Q轴电流环PID */
static foc_t foc_handle;         /* FOC控制句柄 */
static volatile uint8_t current_loop_enable = 0;  /* 电流环使能标志 */
static float target_iq_ramp = 0.0f;  /* Iq目标电流斜坡值 */
static float target_iq_final = 0.0f; /* Iq最终目标电流 */
/* 供打印使用：在 ISR 中写入，在主循环打印，避免在 ISR 中阻塞 */
static volatile float current_d = 0.0f;
static volatile float current_q = 0.0f;

/* 电流双闭环测试函数 */
void test_current_closed_loop(void)
{
    /* 初始化PID控制器 - 降低参数和限幅防止过冲 */
    /* 电流环输出限幅不超过Udc/√3，避免过调制 */
    float v_limit = U_DC * 0.557f;  /* 约6.75V */
    pid_init(&pid_id, 0.67f, 0.000035f, 0.0f, v_limit, -v_limit);
    pid_init(&pid_iq, 0.67f, 0.000035f, 0.0f, v_limit, -v_limit);

    /* 初始化FOC句柄 */
    foc_init(&foc_handle, &pid_id, &pid_iq, NULL);

    /* 电机零点对齐 */
    foc_alignment(&foc_handle);

    /* 软启动：初始目标电流为0 */
    target_iq_final = 0.1f;
    target_iq_ramp = 0.0f;
    foc_set_target(&foc_handle, 0.0f, 0.0f, 0.0f);

    printf("Current Closed Loop Test Start!\r\n");
    printf("Target Id: %.2f A, Target Iq: %.2f A\r\n", foc_handle.target_Id, target_iq_final);

    /* 使能电流环 */
    current_loop_enable = 1;

    /* 主循环：等待按键退出 */
    while (1)
    {
        /* 延时，避免CPU占用过高 */
        delay_us(100);

        /* 每 100ms 打印一次当前 dq 电流 */
        static uint32_t last_print_tick = 0;
        if (HAL_GetTick() - last_print_tick >= 100)
        {
            last_print_tick = HAL_GetTick();
            if (current_loop_enable)
            {
                printf("dq current:%.3f, %.3f\r\n", current_d, current_q);
            }
        }

        /* 按键检测，按下退出 */
        if (key_scan() == 1)
        {
            /* 关闭电流环 */
            current_loop_enable = 0;

            /* 停止PWM输出 */
            tim1_set_pwm_duty(0, 0, 0);

            /* 复位PID */
            pid_reset(&pid_id);
            pid_reset(&pid_iq);

            break;
        }
        /* no-op */
    }

    printf("Current Closed Loop Test Stop!\r\n");
}

/* ADC注入组中断回调中调用的电流闭环处理函数 */
void current_closed_loop_handler(void)
{
    if (!current_loop_enable)
    {
        return;
    }

    /* 软启动斜坡：逐渐增加目标电流 */
    if (target_iq_ramp < target_iq_final)
    {
        target_iq_ramp += 0.0001f;  /* 斜坡增量，根据PWM频率调整 */
        if (target_iq_ramp > target_iq_final)
        {
            target_iq_ramp = target_iq_final;
        }
        foc_handle.target_Iq = target_iq_ramp;
    }

    /* 获取ADC注入组转换值 */
    adc_values_t adc_values;
    adc1_get_injected_values(&adc_values);

    /* 获取电角度 */
    float angle_mech = as5047_get_angle_rad();  /* 机械角度 (rad) */
    float angle_el = (angle_mech - foc_handle.angle_offset) * MOTOR_POLE_PAIR;  /* 电角度 (rad) */

    // /* 角度归一化到 [0, 2π] */
    // while (angle_el < 0.0f)
    // {
    //     angle_el += 2.0f * M_PI;
    // }
    // while (angle_el >= 2.0f * M_PI)
    // {
    //     angle_el -= 2.0f * M_PI;
    // }

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

    /* 电流闭环控制 */
    foc_current_closed_loop(&foc_handle, i_dq, angle_el);


}
