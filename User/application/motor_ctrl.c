#include "motor_ctrl.h"
#include "bsp/clock.h"
#include "bsp/usart.h"
#include "bsp/key.h"
#include "bsp/led.h"
#include "bsp/adc.h"
#include "bsp/tim.h"
#include "bsp/as5047.h"
#include "foc/clark_park.h"
#include "foc/svpwm.h"
#include "utils/print.h"
#include <stdio.h>
#include <math.h>

/* FOC 控制句柄 */
static foc_t foc_handle;

/* PID 控制器实例 */
static pid_controller_t pid_id;
static pid_controller_t pid_iq;
static pid_controller_t pid_speed;

/* 当前控制模式 */
static volatile ctrl_mode_t ctrl_mode = CTRL_MODE_IDLE;

/* 实际 dq 电流反馈 (打印用) */
static volatile dq_t i_dq_feedback = {0};

/* FOC 主控制adc注入中断回调 - 10kHz */
static void motor_ctrl_callback(void)
{
    /* 更新编码器速度 */
    as5047_update_speed();

    /* 空闲模式不执行 */
    if (ctrl_mode == CTRL_MODE_IDLE)
        return;
        
    /* 开环模式单独处理 */
    if (ctrl_mode == CTRL_MODE_OPEN_LOOP)
    {
        foc_open_loop_run();
        return;
    }

    /* 获取 ADC 采样值 */
    adc_values_t adc_values;
    adc1_get_injected_values(&adc_values);

    /* 计算电角度 */
    float mech_angle = as5047_get_angle_rad();
    float angle_el = mech_angle * MOTOR_POLE_PAIR - foc_handle.angle_offset;

    /* 归一化到 [0, 2π) */
    while (angle_el >= 2.0f * M_PI)
        angle_el -= 2.0f * M_PI;
    while (angle_el < 0.0f)
        angle_el += 2.0f * M_PI;

    /* Clark 变换: abc -> αβ */
    abc_t i_abc = {.a = adc_values.ia, .b = adc_values.ib, .c = adc_values.ic};
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    /* Park 变换: αβ -> dq */
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    /* 保存电流反馈供打印 */
    i_dq_feedback = i_dq;

    /* 执行闭环控制 */
    if (ctrl_mode == CTRL_MODE_CURRENT)
    {
        foc_current_closed_loop(&foc_handle, i_dq, angle_el);
    }
    else if (ctrl_mode == CTRL_MODE_SPEED)
    {
        foc_speed_closed_loop(&foc_handle, i_dq, angle_el, as5047_get_speed_rpm());
    }
}

void motor_ctrl_bsp_init(void)
{
    HAL_Init();
    clock_init();
    usart1_init();

    led1_init();
    key_init();
    led2_init();

    as5047_init();
    tim3_init();
    tim1_init();
    adc1_init();
}

void motor_ctrl_init(void)
{
    /* 电流环 PID: Kp=0.017, Ki=0.002826, 输出限幅 ±U_DC/2 */
    pid_init(&pid_id, 0.017f, 0.002826f, 0.0f, U_DC / 2.0f, -U_DC / 2.0f);
    pid_init(&pid_iq, 0.017f, 0.002826f, 0.0f, U_DC / 2.0f, -U_DC / 2.0f);

    /* 速度环 PID: Kp=0.01, Ki=0.001, 输出限幅 ±2A */
    pid_init(&pid_speed, 0.05f, 0.00002f, 0.0f, 2.0f, -2.0f);

    /* 初始化 FOC 控制句柄 */
    foc_init(&foc_handle, &pid_id, &pid_iq, &pid_speed);

    /* 注册 ADC 注入组中断回调 */
    adc1_register_injected_callback(motor_ctrl_callback);

    /* 电机零点对齐 */
    foc_alignment(&foc_handle);

    ctrl_mode = CTRL_MODE_IDLE;
}

void motor_ctrl_switch_mode(void)
{
    switch (ctrl_mode)
    {
    case CTRL_MODE_IDLE:
        ctrl_mode = CTRL_MODE_OPEN_LOOP;
        foc_open_loop_set(100.0f, 1.0f); /* 默认 100RPM, 1V */
        printf("Mode: OPEN LOOP, 100RPM\n");
        break;

    case CTRL_MODE_OPEN_LOOP:
        foc_open_loop_stop();
        ctrl_mode = CTRL_MODE_CURRENT;
        foc_handle.target_id = 0.0f;
        foc_handle.target_iq = 0.5f;
        printf("Mode: CURRENT, Iq=0.5A\n");
        break;

    case CTRL_MODE_CURRENT:
        ctrl_mode = CTRL_MODE_SPEED;
        foc_handle.target_speed = 300.0f;
        printf("Mode: SPEED, 300RPM\n");
        break;

    case CTRL_MODE_SPEED:
        motor_ctrl_stop();
        printf("Mode: IDLE\n");
        break;
    }
}

void motor_ctrl_stop(void)
{
    ctrl_mode = CTRL_MODE_IDLE;
    foc_open_loop_stop();
    foc_closed_loop_stop(&foc_handle);
}

void motor_ctrl_print_status(void)
{
    if (ctrl_mode == CTRL_MODE_IDLE)
        return;

    printf_period(500, "RPM, Id, Iq: %.1f, %.2f, %.2f\n", as5047_get_speed_rpm_lpf(), i_dq_feedback.d, i_dq_feedback.q);
}
