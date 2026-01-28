#include "luenberger.h"

void luenberger_init(luenberger_t *luenberger, float rs, float ls, float poles, float ts, float l1, float l2, float pll_fc, float k_speed_lpf)
{
    // 保存电机参数
    luenberger->rs = rs;
    luenberger->ls = ls;
    luenberger->poles = poles;
    luenberger->ts = ts;

    // 可调参数初始化
    luenberger->l1 = l1;
    luenberger->l2 = l2;
    luenberger->k_speed_lpf = k_speed_lpf;

    // 状态清零
    luenberger->i_alpha_est = 0.0f;
    luenberger->i_beta_est = 0.0f;
    luenberger->e_alpha_est = 0.0f;
    luenberger->e_beta_est = 0.0f;

    luenberger->theta_est = 0.0f;
    luenberger->speed_est = 0.0f;
    luenberger->speed_est_filt = 0.0f;
    luenberger->speed_rad_s = 0.0f;

    // PLL 初始化
    // 典型值：Kp = 2 * ζ * ωn, Ki = ωn^2
    // ωn = 2π * fc
    float wn = 2 * 3.14159 * pll_fc;
    float zeta = 1.0f; // 阻尼系数
    float kp = 2.0f * zeta * wn;
    float ki = wn * wn * ts; // 注意：pid_calculate 内部不乘 ts，所以这里预乘

    luenberger->k_pll_kp = kp;
    luenberger->k_pll_ki = ki;

    // 估算最大转速用于限幅
    float max_rpm = 10000.0f;
    float max_speed_rad_s = max_rpm * 2.0f * 3.14159265f * poles / 60.0f;

    pid_init(&luenberger->pll, kp, ki, -max_speed_rad_s, max_speed_rad_s);
}

void luenberger_estimate(luenberger_t *luenberger)
{
    // 提取电机参数
    float ts = luenberger->ts;
    float rs = luenberger->rs;
    float ls = luenberger->ls;

    // 获取当前时刻(k)的状态变量
    float i_alpha_k = luenberger->i_alpha_est;
    float i_beta_k = luenberger->i_beta_est;
    float e_alpha_k = luenberger->e_alpha_est;
    float e_beta_k = luenberger->e_beta_est;

    // 输入变量
    float u_alpha = luenberger->u_alpha;
    float u_beta = luenberger->u_beta;
    float i_alpha_meas = luenberger->i_alpha;
    float i_beta_meas = luenberger->i_beta;

    // 使用上一时刻估计的电角速度
    float we = luenberger->speed_rad_s;

    // 计算电流估算误差: i_est(k) - i_meas(k)
    float i_err_alpha = i_alpha_k - i_alpha_meas;
    float i_err_beta = i_beta_k - i_beta_meas;

    float k_tl = ts / ls;
    float k_trl = ts * rs / ls;

    // 电流观测器更新
    float i_alpha_next = i_alpha_k - k_trl * i_alpha_k - k_tl * e_alpha_k + k_tl * u_alpha + luenberger->l1 * ts * i_err_alpha;
    float i_beta_next = i_beta_k - k_trl * i_beta_k - k_tl * e_beta_k + k_tl * u_beta + luenberger->l1 * ts * i_err_beta;

    // 反电势观测器更新
    float e_alpha_next = e_alpha_k - we * ts * e_beta_k + luenberger->l2 * ts * i_err_alpha;
    float e_beta_next = e_beta_k + we * ts * e_alpha_k + luenberger->l2 * ts * i_err_beta;

    // 更新状态
    luenberger->i_alpha_est = i_alpha_next;
    luenberger->i_beta_est = i_beta_next;
    luenberger->e_alpha_est = e_alpha_next;
    luenberger->e_beta_est = e_beta_next;

    // --- PLL 锁相环 ---
    float sin_theta, cos_theta;
    fast_sin_cos(luenberger->theta_est, &sin_theta, &cos_theta);

    // 计算 PLL 误差
    float pll_err = -(luenberger->e_alpha_est * cos_theta + luenberger->e_beta_est * sin_theta);

    // PI 计算得到角速度
    luenberger->speed_rad_s = pid_calculate(&luenberger->pll, pll_err, 0.0f);

    // 计算机械转速 (RPM)
    luenberger->speed_est = luenberger->speed_rad_s * 60.0f / (2.0f * 3.14159265f * luenberger->poles);

    // 对速度进行低通滤波
    luenberger->speed_est_filt = (1.0f - luenberger->k_speed_lpf) * luenberger->speed_est_filt + luenberger->k_speed_lpf * luenberger->speed_est;
    
    // 积分得到角度
    luenberger->theta_est += luenberger->speed_rad_s * ts;

    // 角度归一化 (-pi, pi]
    if (luenberger->theta_est > 3.14159265f)
        luenberger->theta_est -= 2.0f * 3.14159265f;
    else if (luenberger->theta_est <= -3.14159265f)
        luenberger->theta_est += 2.0f * 3.14159265f;
}

float luenberger_get_angle(luenberger_t *luenberger)
{
    return luenberger->theta_est;
}

float luenberger_get_speed_rpm(luenberger_t *luenberger)
{
    return luenberger->speed_est_filt;
}
