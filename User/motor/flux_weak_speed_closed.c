#include "flux_weak_speed_closed.h"

static foc_t foc_flux_weak_speed_handle;

// PID 控制器
static pid_controller_t pid_id;
static pid_controller_t pid_iq;
static pid_controller_t pid_speed;

// 打印用
static dq_t i_dq_temp = {
    .d = 0.0f,
    .q = 0.0f,
};
static float speed_rpm_temp = 0.0f;
static float id_target_temp = 0.0f;

// 弱磁速度闭环模式回调
static void flux_weak_speed_closed_callback(void)
{
    // 更新速度
    as5047_update_speed();

    // 获取角度和速度
    float angle_el = as5047_get_angle_rad() - foc_flux_weak_speed_handle.angle_offset;
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

    // 打印用
    i_dq_temp = i_dq;

    // 弱磁速度闭环
    foc_flux_weak_speed_closed_loop_run(&foc_flux_weak_speed_handle, i_dq, angle_el, speed_feedback);

    // 打印用
    id_target_temp = foc_flux_weak_speed_handle.target_id;
}

void flux_weak_speed_closed_init(float speed_rpm)
{
    // 初始化 PID 控制器
    pid_init(&pid_id, 0.017f, 0.002826f, -U_DC / 3.0f, U_DC / 3.0f);
    pid_init(&pid_iq, 0.017f, 0.002826f, -U_DC / 3.0f, U_DC / 3.0f);
    pid_init(&pid_speed, 0.005f, 0.000002f, -2.0f, 2.0f);

    // 初始化 FOC 控制句柄
    foc_init(&foc_flux_weak_speed_handle, &pid_id, &pid_iq, &pid_speed);

    // 设置目标值
    foc_set_target_id(&foc_flux_weak_speed_handle, 0.0f);
    foc_set_target_speed(&foc_flux_weak_speed_handle, speed_rpm);

    // 零点对齐
    foc_alignment(&foc_flux_weak_speed_handle);

    // 注册回调函数
    adc1_register_injected_callback(flux_weak_speed_closed_callback);
}

void print_flux_weak_speed_info(void)
{
    float data[4] = {speed_rpm_temp, i_dq_temp.d, i_dq_temp.q, id_target_temp};
    printf_vofa(data, 4);
}
