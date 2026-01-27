#include "smo.h"
#include "pid.h"

// 滑模控制率 - 饱和函数
static float smo_fun(float error, float boundary)
{
    // 饱和函数：在边界层内线性，边界层外饱和
    if (error > boundary)
        return 1.0f;
    else if (error < -boundary)
        return -1.0f;
    else
        return error / boundary; // 边界层内线性过渡
}

void smo_init(smo_t *smo, float rs, float ls, float poles, float ts, float k_slide, float k_lpf, float boundary, float fc, float k_speed_lpf)
{
    // 电机参数
    smo->rs = rs;
    smo->ls = ls;
    smo->poles = poles;
    smo->ts = ts;

    // 可调参数
    smo->k_slide = k_slide;
    smo->k_lpf = k_lpf;
    smo->boundary = boundary;
    smo->k_speed_lpf = k_speed_lpf;

    // PLL 参数 - 使用 pid_init 初始化
    // 典型值：Kp = 2 * ζ * ωn, Ki = ωn^2
    // ωn = 2π * fc
    float wn = 2.0f * 3.14159265f * fc;
    float zeta = 1.0f; // 阻尼系数
    float kp = 2.0f * zeta * wn;
    float ki = wn * wn * ts; // 注意：pid_calculate 内部不乘 ts，所以这里预乘

    // 速度范围：假设最大 ±10000 RPM
    // 转换为电角速度：ω_elec = RPM * 2π * poles / 60
    float max_rpm = 10000.0f;
    float max_speed_rad_s = max_rpm * 2.0f * 3.14159265f * poles / 60.0f;

    pid_init(&smo->pll, kp, ki, -max_speed_rad_s, max_speed_rad_s);

    // 初始化状态
    smo->i_alpha = 0.0f;
    smo->i_beta = 0.0f;
    smo->u_alpha = 0.0f;
    smo->u_beta = 0.0f;

    smo->i_alpha_est = 0.0f;
    smo->i_beta_est = 0.0f;
    smo->e_alpha = 0.0f;
    smo->e_beta = 0.0f;
    smo->z_alpha = 0.0f;
    smo->z_beta = 0.0f;

    smo->theta_est = 0.0f;
    smo->theta_comp = 0.0f;
    smo->speed_est = 0.0f;
    smo->speed_est_filt = 0.0f;
}

void smo_estimate(smo_t *smo)
{
    // 中间变量计算
    float F = 1.0f - smo->rs * smo->ts / smo->ls;
    float G = smo->ts / smo->ls;

    // 先更新 (k+1) 时刻电流估计值
    smo->i_alpha_est = F * smo->i_alpha_est + G * (smo->u_alpha - smo->e_alpha - smo->z_alpha);
    smo->i_beta_est = F * smo->i_beta_est + G * (smo->u_beta - smo->e_beta - smo->z_beta);

    // 用最新的估计值计算误差
    float i_err_alpha = smo->i_alpha_est - smo->i_alpha;
    float i_err_beta = smo->i_beta_est - smo->i_beta;

    // 计算滑模控制量
    smo->z_alpha = smo->k_slide * smo_fun(i_err_alpha, smo->boundary);
    smo->z_beta = smo->k_slide * smo_fun(i_err_beta, smo->boundary);

    // 低通滤波得到反电势估计
    smo->e_alpha = (1 - smo->k_lpf) * smo->e_alpha + smo->k_lpf * smo->z_alpha;
    smo->e_beta = (1 - smo->k_lpf) * smo->e_beta + smo->k_lpf * smo->z_beta;

    // 利用 PLL 估算角度和速度
    float sin_theta, cos_theta;
    fast_sin_cos(smo->theta_est, &sin_theta, &cos_theta);

    // PLL 误差计算
    // ΔE = -Êα × cos(θ̂e) - Êβ × sin(θ̂e)
    float pll_err = -(smo->e_alpha * cos_theta + smo->e_beta * sin_theta);

    // 使用 PI 控制器调节速度
    float speed_rad_s = pid_calculate(&smo->pll, pll_err, 0.0f);

    // 转换为机械转速 RPM: ω_mech = ω_elec / poles
    smo->speed_est = speed_rad_s * 60.0f / (2.0f * 3.14159265f * smo->poles);

    // 对速度进行低通滤波
    smo->speed_est_filt = (1.0f - smo->k_speed_lpf) * smo->speed_est_filt + smo->k_speed_lpf * smo->speed_est;

    // 积分速度得到角度（rad/s）
    smo->theta_est += speed_rad_s * smo->ts;

    // 角度归一化到 [0, 2π]
    const float TWO_PI = 2.0f * 3.14159265f;
    while (smo->theta_est >= TWO_PI)
        smo->theta_est -= TWO_PI;
    while (smo->theta_est < 0.0f)
        smo->theta_est += TWO_PI;

    // 计算低通滤波带来的相位滞后并进行补偿
    float omega_e = smo->speed_est_filt * smo->poles * TWO_PI / 60.0f;
    float delta_theta = atanf((omega_e * smo->ts * (1.0f - smo->k_lpf)) / smo->k_lpf);

    smo->theta_comp = smo->theta_est + delta_theta;

    // 补偿后的角度归一化
    while (smo->theta_comp >= TWO_PI)
        smo->theta_comp -= TWO_PI;
    while (smo->theta_comp < 0.0f)
        smo->theta_comp += TWO_PI;
}

float smo_get_bemf_alpha(smo_t *smo)
{
    return smo->e_alpha;
}

float smo_get_bemf_beta(smo_t *smo)
{
    return smo->e_beta;
}

float smo_get_angle(smo_t *smo)
{
    return smo->theta_comp;
}

float smo_get_speed_rpm(smo_t *smo)
{
    return smo->speed_est_filt; // 返回滤波后的速度
}