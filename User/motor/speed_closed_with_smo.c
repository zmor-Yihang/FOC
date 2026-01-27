#include "speed_closed_with_smo.h"

// FOC 控制句柄
static foc_t foc_handle;

// SMO 观测器实例（仅用于观测，不参与控制）
static smo_t smo;

// PID 实例
static pid_controller_t pid_id;
static pid_controller_t pid_iq;
static pid_controller_t pid_speed;

// 打印用变量
static float speed_rpm_encoder = 0.0f;
static float angle_el_encoder = 0.0f;
static float speed_rpm_smo = 0.0f;
static float angle_el_smo = 0.0f;
static float bemf_alpha_smo = 0.0f;
static float bemf_beta_smo = 0.0f;

static void speed_closed_with_smo_callback(void)
{
    // 更新编码器速度
    as5047_update_speed();

    // 获取编码器角度和速度（用于控制）
    float angle_el = as5047_get_angle_rad() - foc_handle.angle_offset;
    float speed_feedback = as5047_get_speed_rpm();

    // 获取电流反馈值
    adc_values_t adc_values;
    adc1_get_injected_values(&adc_values);

    // Clark 变换
    abc_t i_abc = {.a = adc_values.ia, .b = adc_values.ib, .c = adc_values.ic};
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    // Park 变换（使用编码器角度）
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    // 速度闭环控制（使用编码器反馈）
    foc_speed_closed_loop_run(&foc_handle, i_dq, angle_el, speed_feedback);

    // 获取输出电压
    dq_t v_dq = {.d = foc_handle.v_d_out, .q = foc_handle.v_q_out};

    // 反Park变换（使用编码器角度）
    alphabeta_t v_alphabeta = ipark_transform(v_dq, angle_el);

    // 更新SMO观测器（仅用于观测对比）
    smo.i_alpha = i_alphabeta.alpha;
    smo.i_beta = i_alphabeta.beta;
    smo.u_alpha = v_alphabeta.alpha;
    smo.u_beta = v_alphabeta.beta;
    smo_estimate(&smo);

    // 获取SMO观测值
    float angle_smo = smo_get_angle(&smo);
    float speed_smo = smo_get_speed_rpm(&smo);

    // 保存打印数据
    speed_rpm_encoder = speed_feedback;
    angle_el_encoder = angle_el;
    speed_rpm_smo = speed_smo;
    angle_el_smo = angle_smo;
    bemf_alpha_smo = smo_get_bemf_alpha(&smo);
    bemf_beta_smo = smo_get_bemf_beta(&smo);
}

void speed_closed_with_smo_init(float speed_rpm)
{
    // PID 初始化
    pid_init(&pid_id, 0.017f, 0.002826f, -U_DC / 3.0f, U_DC / 3.0f);
    pid_init(&pid_iq, 0.017f, 0.002826f, -U_DC / 3.0f, U_DC / 3.0f);
    pid_init(&pid_speed, 0.05f, 0.00002f, -4.0f, 4.0f);

    // FOC 初始化
    foc_init(&foc_handle, &pid_id, &pid_iq, &pid_speed);

    // 初始化 SMO 观测器
    smo_init(&smo, 0.12f, 0.0003f, 7.0f, 0.0001f,
             0.6f,   // k_slide - 滑模增益
             0.1f,   // k_lpf - 低通滤波系数
             3.0f,   // boundary - 边界层厚度
             200.0f, // fc - PLL截止频率
             0.05f); // k_speed_lpf - 速度滤波系数

    // 设置目标值
    foc_set_target_id(&foc_handle, 0.0f);
    foc_set_target_speed(&foc_handle, speed_rpm);

    // 零点对齐
    foc_alignment(&foc_handle);

    // 注册回调函数
    adc1_register_injected_callback(speed_closed_with_smo_callback);
}

void print_speed_smo_info(void)
{
    // 归一化角度到 [0, 2π) 范围
    float angle_encoder_normalized = fmodf(angle_el_encoder, 2.0f * M_PI);
    if (angle_encoder_normalized < 0.0f)
    {
        angle_encoder_normalized += 2.0f * M_PI;
    }

    float angle_smo_normalized = fmodf(angle_el_smo, 2.0f * M_PI);
    if (angle_smo_normalized < 0.0f)
    {
        angle_smo_normalized += 2.0f * M_PI;
    }

    // 转换为角度 (0-360°)
    float angle_encoder_deg = angle_encoder_normalized * 57.2958f;
    float angle_smo_deg = angle_smo_normalized * 57.2958f;

    // 发送 JustFloat 协议数据
    float data[6] = {speed_rpm_encoder, speed_rpm_smo, angle_encoder_deg, angle_smo_deg, bemf_alpha_smo, bemf_beta_smo};
    printf_vofa(data, 6);
}
