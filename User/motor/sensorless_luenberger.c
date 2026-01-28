#include "sensorless_luenberger.h"

// FOC 控制句柄
static foc_t foc_luenberger_handle;

// Luenberger 观测器实例
static luenberger_t luenberger;

// pid 实例
static pid_controller_t pid_id;
static pid_controller_t pid_iq;
static pid_controller_t pid_speed;

// 状态变量
static luenberger_state_t current_state = LUENBERGER_STATE_IF_STARTUP;
static uint32_t switch_counter = 0;

// 斜坡加速变量
static float target_speed_ramp = 0.0f;
static const float RAMP_RATE = 100.0f; // 加速度: 100 RPM/s
static const float DT = 0.001f;        // 控制周期: 1ms

// 打印用
static float speed_rpm_actual_temp = 0.0f;
static float angle_el_actual_temp = 0.0f;
static float speed_rpm_luenberger_temp = 0.0f;
static float angle_el_luenberger_temp = 0.0f;

static void sensorless_luenberger_callback(void)
{
    // 获取电流反馈值
    adc_values_t adc_values;
    adc1_get_injected_values(&adc_values);

    // Clark 变换
    abc_t i_abc = {.a = adc_values.ia, .b = adc_values.ib, .c = adc_values.ic};
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    // 获取Luenberger观测的电角度和速度
    float angle_el_luenberger = luenberger_get_angle(&luenberger);
    float speed_feedback_luenberger = luenberger_get_speed_rpm(&luenberger);

    // 根据状态选择角度源
    float angle_for_control;
    if (current_state == LUENBERGER_STATE_IF_STARTUP)
    {
        angle_for_control = foc_luenberger_handle.open_loop_angle_el; // IF角度
    }
    else
    {
        angle_for_control = angle_el_luenberger; // Luenberger角度
    }

    // Park 变换 - 使用选定的角度
    dq_t i_dq = park_transform(i_alphabeta, angle_for_control);

    // 根据状态执行不同的控制
    if (current_state == LUENBERGER_STATE_IF_STARTUP)
    {
        // 斜坡加速到目标速度
        target_speed_ramp = ramp_update(target_speed_ramp, 200.0f, RAMP_RATE, DT);
        
        // I/F 电流开环 - 使用斜坡速度
        foc_if_current_run(&foc_luenberger_handle, i_dq, target_speed_ramp, 0.5f);
    }
    else
    {
        // 速度闭环
        foc_speed_closed_loop_run(&foc_luenberger_handle, i_dq, angle_for_control, speed_feedback_luenberger);
    }

    // 获取Vd和Vq
    dq_t v_dq = {.d = foc_luenberger_handle.v_d_out, .q = foc_luenberger_handle.v_q_out};

    // 反Park变换 - 使用选定的角度
    alphabeta_t v_alphabeta = ipark_transform(v_dq, angle_for_control);

    // 更新Luenberger
    luenberger.i_alpha = i_alphabeta.alpha;
    luenberger.i_beta = i_alphabeta.beta;
    luenberger.u_alpha = v_alphabeta.alpha;
    luenberger.u_beta = v_alphabeta.beta;
    luenberger_estimate(&luenberger);

    // 切换逻辑
    if (current_state == LUENBERGER_STATE_IF_STARTUP)
    {
        float speed_error = fabsf(200 - speed_feedback_luenberger);

        if (speed_error < 50.0f)
        {
            switch_counter++;
            if (switch_counter > 2000)
            { // 保持2000个周期(约2000ms)
                current_state = LUENBERGER_STATE_RUNNING;
                switch_counter = 0;
            }
        }
        else
        {
            switch_counter = 0; // 不满足条件则清零
        }
    }

    // 打印
    as5047_update_speed();
    speed_rpm_actual_temp = as5047_get_speed_rpm();
    angle_el_actual_temp = as5047_get_angle_rad() - foc_luenberger_handle.angle_offset;
    speed_rpm_luenberger_temp = speed_feedback_luenberger;
    angle_el_luenberger_temp = angle_el_luenberger;
}

void sensorless_luenberger_init(float speed_rpm)
{
    // pid 初始化
    pid_init(&pid_id, 0.017f, 0.002826f, -U_DC / 3.0f, U_DC / 3.0f);
    pid_init(&pid_iq, 0.017f, 0.002826f, -U_DC / 3.0f, U_DC / 3.0f);
    pid_init(&pid_speed, 0.005f, 0.000002f, -2.0f, 2.0f);

    foc_init(&foc_luenberger_handle, &pid_id, &pid_iq, &pid_speed);

    // 初始化 Luenberger 观测器
    luenberger_init(&luenberger, 0.12f, 0.00003f, 7.0f, 0.0001f,
                    -13000.0f, // l1
                    2200.0f,   // l2
                    50.0f,    // pll_fc: 增大PLL带宽，提高动态响应
                    0.05f);    // k_speed_lpf

    foc_set_target_id(&foc_luenberger_handle, 0.0f);

    foc_set_target_speed(&foc_luenberger_handle, speed_rpm);

    // 初始化状态
    current_state = LUENBERGER_STATE_IF_STARTUP;
    switch_counter = 0;
    target_speed_ramp = 0.0f; // 初始化斜坡速度为0

    // 零点对齐
    foc_alignment(&foc_luenberger_handle);

    adc1_register_injected_callback(sensorless_luenberger_callback);
}

void print_sensorless_luenberger_info(void)
{
    // 归一化角度到 [0, 2π) 范围
    float angle_actual_normalized = fmodf(angle_el_actual_temp, 2.0f * M_PI);
    if (angle_actual_normalized < 0.0f)
    {
        angle_actual_normalized += 2.0f * M_PI;
    }

    float angle_luenberger_normalized = fmodf(angle_el_luenberger_temp, 2.0f * M_PI);
    if (angle_luenberger_normalized < 0.0f)
    {
        angle_luenberger_normalized += 2.0f * M_PI;
    }

    // 转换为角度 (0-360°)
    float angle_actual_deg = angle_actual_normalized * 57.2958f;
    float angle_luenberger_deg = angle_luenberger_normalized * 57.2958f;

    float data[4] = {speed_rpm_actual_temp, angle_actual_deg, speed_rpm_luenberger_temp, angle_luenberger_deg};
    printf_vofa(data, 4);
}
