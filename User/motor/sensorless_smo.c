#include "sensorless_smo.h"

// FOC 控制句柄
static foc_t foc_smo_handle;

// SMO 观测器实例
static smo_t smo;

// pid 实例
static pid_controller_t pid_id;
static pid_controller_t pid_iq;
static pid_controller_t pid_speed;

// 状态变量
static sensorless_state_t current_state = STATE_IF_STARTUP;
static uint32_t switch_counter = 0;

// 打印用
static float speed_rpm_actual_temp = 0.0f;
static float angle_el_actual_temp = 0.0f;
static float speed_rpm_smo_temp = 0.0f;
static float angle_el_smo_temp = 0.0f;

static void sensorless_smo_callback(void)
{
    // 获取电流反馈值
    adc_values_t adc_values;
    adc1_get_injected_values(&adc_values);

    // Clark 变换
    abc_t i_abc = {.a = adc_values.ia, .b = adc_values.ib, .c = adc_values.ic};
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    // 获取SMO观测的电角度和速度
    float angle_el_smo = smo_get_angle(&smo);
    float speed_feedback_smo = smo_get_speed_rpm(&smo);

    // 根据状态选择角度源
    float angle_for_control;
    if (current_state == STATE_IF_STARTUP)
    {
        angle_for_control = foc_smo_handle.open_loop_angle_el; // IF角度
    }
    else
    {
        angle_for_control = angle_el_smo; // SMO角度
    }

    // Park 变换 - 使用选定的角度
    dq_t i_dq = park_transform(i_alphabeta, angle_for_control);

    // 根据状态执行不同的控制
    if (current_state == STATE_IF_STARTUP)
    {
        // I/F 电流开环
        foc_if_current_run(&foc_smo_handle, i_dq, 200.0f, 0.5f);
    }
    else
    {
        // 速度闭环
        foc_speed_closed_loop_run(&foc_smo_handle, i_dq, angle_for_control, speed_feedback_smo);
    }

    // 获取Vd和Vq
    dq_t v_dq = {.d = foc_smo_handle.v_d_out, .q = foc_smo_handle.v_q_out};

    // 反Park变换 - 使用选定的角度
    alphabeta_t v_alphabeta = ipark_transform(v_dq, angle_for_control);

    // 更新SMO
    smo.i_alpha = i_alphabeta.alpha;
    smo.i_beta = i_alphabeta.beta;
    smo.u_alpha = v_alphabeta.alpha;
    smo.u_beta = v_alphabeta.beta;
    smo_estimate(&smo);

    // 切换逻辑
    if (current_state == STATE_IF_STARTUP)
    {
        float speed_error = fabsf(200 - speed_feedback_smo);

        if (speed_error < 50.0f)
        {
            switch_counter++;
            if (switch_counter > 100)
            { // 持续100个周期(约100ms)
                current_state = STATE_SMO_RUNNING;
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
    angle_el_actual_temp = as5047_get_angle_rad() - foc_smo_handle.angle_offset;
    speed_rpm_smo_temp = speed_feedback_smo;
    angle_el_smo_temp = angle_el_smo;
}

void sensorless_smo_init(float speed_rpm)
{
    // pid 初始化
    pid_init(&pid_id, 0.017f, 0.002826f, -U_DC / 2.0f, U_DC / 2.0f);
    pid_init(&pid_iq, 0.017f, 0.002826f, -U_DC / 2.0f, U_DC / 2.0f);
    pid_init(&pid_speed, 0.005f, 0.000002f, -2.0f, 2.0f);

    foc_init(&foc_smo_handle, &pid_id, &pid_iq, &pid_speed);
    
    // 初始化 SMO 观测器
    smo_init(&smo, 0.12f, 0.0003f, 7.0f, 0.0001f,
             1.4f,   // k_slide - 滑模增益
             0.4f,   // k_lpf - 低通滤波系数
             3.0f,   // boundary - 边界层厚度
             100.0f, // fc - PLL截止频率
             0.95f); // k_speed_lpf - 速度滤波系数

    foc_set_target_id(&foc_smo_handle, 0.0f);

    foc_set_target_speed(&foc_smo_handle, speed_rpm);

    // 初始化状态
    current_state = STATE_IF_STARTUP;
    switch_counter = 0;

    // 零点对齐
    foc_alignment(&foc_smo_handle);

    adc1_register_injected_callback(sensorless_smo_callback);
}

void print_sensorless_info(void)
{
    // 归一化角度到 [0, 2π) 范围
    float angle_actual_normalized = fmodf(angle_el_actual_temp, 2.0f * M_PI);
    if (angle_actual_normalized < 0.0f)
    {
        angle_actual_normalized += 2.0f * M_PI;
    }

    float angle_smo_normalized = fmodf(angle_el_smo_temp, 2.0f * M_PI);
    if (angle_smo_normalized < 0.0f)
    {
        angle_smo_normalized += 2.0f * M_PI;
    }

    // 转换为角度 (0-360°)
    float angle_actual_deg = angle_actual_normalized * 57.2958f;
    float angle_smo_deg = angle_smo_normalized * 57.2958f;

    printf("%.2f, %.2f, %.2f, %.2f\n", speed_rpm_actual_temp, angle_actual_deg, speed_rpm_smo_temp, angle_smo_deg);
}