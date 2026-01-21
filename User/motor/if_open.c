#include "if_open.h"

static foc_t foc_if_open_handle;

static pid_controller_t pid_id;
static pid_controller_t pid_iq;

// 目标转速
static float speed_rpm_target = 0.0f;  // 目标速度

// 打印用
static dq_t i_dq_temp = {
    .d = 0.0f,
    .q = 0.0f,
};

// I/F 开环模式回调
static void if_open_callback(void)
{
    // 获取电流反馈值
    adc_values_t adc_values;
    adc1_get_injected_values(&adc_values);

    // Clark 变换
    abc_t i_abc = {.a = adc_values.ia, .b = adc_values.ib, .c = adc_values.ic};
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    // Park 变换 - 使用 I/F 角度
    dq_t i_dq = park_transform(i_alphabeta, foc_if_open_handle.open_loop_angle_el);


    // 打印用
    i_dq_temp = i_dq;

    // I/F 电流开环
    foc_if_current_run(&foc_if_open_handle, i_dq, speed_rpm_target, foc_if_open_handle.target_iq);
}

void if_open_init(float speed_rpm, float iq)
{
    // 初始化电流环 PID 控制器
    pid_init(&pid_id, 0.017f, 0.002826f, -U_DC / 2.0f, U_DC / 2.0f);
    pid_init(&pid_iq, 0.017f, 0.002826f, -U_DC / 2.0f, U_DC / 2.0f);

    // 初始化 FOC 控制句柄
    foc_init(&foc_if_open_handle, &pid_id, &pid_iq, NULL);

    // 设置目标速度和电流
    speed_rpm_target = speed_rpm;
    foc_set_target_iq(&foc_if_open_handle, iq);

    // 零点对齐
    foc_alignment(&foc_if_open_handle);

    // 注册回调函数
    adc1_register_injected_callback(if_open_callback);
}

void print_if_current_info(void)
{
    printf("%.2f, %.2f\n", i_dq_temp.d, i_dq_temp.q);
}
