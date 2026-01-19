#include "speed_closed.h"

static foc_t foc_speed_closed_handle;

static pid_controller_t pid_id;
static pid_controller_t pid_iq;

static pid_controller_t pid_speed;

// 打印用
static float speed_rpm_temp = 0.0f;

static void speed_closed_callback(void)
{
    // 更新速度
    as5047_update_speed();

    // 获取角度和速度
    float angle_el = as5047_get_angle_rad() - foc_speed_closed_handle.angle_offset;
    float speed_feedback = as5047_get_speed_rpm();

    // 打印用
    speed_rpm_temp = speed_feedback;

    // 获取电流反馈值
    adc_values_t adc_values;
    adc1_get_injected_values(&adc_values);

    // Clark 变换
    abc_t i_abc = {.a = adc_values.ia, .b = adc_values.ib, .c = adc_values.ic};
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    // Park 变换
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    // 速度闭环
    foc_speed_closed_loop_run(&foc_speed_closed_handle, i_dq, angle_el, speed_feedback);
}

void speed_closed_init(float speed_rpm)
{
    // 初始化速度环 PID 控制器
    pid_init(&pid_id, 0.017f, 0.002826f, -U_DC / 2.0f, U_DC / 2.0f);
    pid_init(&pid_iq, 0.017f, 0.002826f, -U_DC / 2.0f, U_DC / 2.0f);
    pid_init(&pid_speed, 0.05f, 0.00002f, -4.0f, 4.0f);

    // 初始化 FOC 控制句柄
    foc_init(&foc_speed_closed_handle, &pid_id, &pid_iq, &pid_speed);

    // 设置目标速度
    foc_set_target_id(&foc_speed_closed_handle, 0.0f);
    foc_set_target_speed(&foc_speed_closed_handle, speed_rpm);

    // 零点对齐
    foc_alignment(&foc_speed_closed_handle);

    // 注册回调函数
    adc1_register_injected_callback(speed_closed_callback);
}

void print_speed_info(void)
{
    printf_period(10, "%.2f\n", speed_rpm_temp);
}