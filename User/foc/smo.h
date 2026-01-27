#ifndef __SMO_H__
#define __SMO_H__

#include <math.h>
#include "utils/fast_sin_cos.h"
#include "pid.h"

// 滑模观测器结构体
typedef struct
{
    // --- 输入 ---
    float i_alpha; // 实测电流 alpha
    float i_beta;  // 实测电流 beta
    float u_alpha; // 以此计算出的电压 alpha
    float u_beta;  // 以此计算出的电压 beta

    // --- 电机参数 ---
    float rs;    // 定子电阻 (Ω)
    float ls;    // 定子电感 (H)
    float ts;    // 控制周期 (s)
    float poles; // 电机极对数

    // --- 可调参数 ---
    float k_slide;  // 滑模增益 (Gain)
    float k_lpf;    // 低通滤波器系数 (0.0 ~ 1.0)
    float boundary; // 边界层厚度

    // 观测状态
    float i_alpha_est; // 估算电流 alpha
    float i_beta_est;  // 估算电流 beta
    float e_alpha;     // 滤波后的反电动势 alpha
    float e_beta;      // 滤波后的反电动势 beta
    float z_alpha;     // 滑模控制量 (Raw BEMF)
    float z_beta;

    // 观测角度和速度
    float theta_est;       // 估算角度 (rad)
    float theta_comp;      // 补偿后的角度 (rad)
    float speed_est;       // 估算速度 (rpm)
    float speed_est_filt;  // 滤波后的速度 (rpm)
    float k_speed_lpf;     // 速度低通滤波系数 (0.0 ~ 1.0)

    // PLL 使用 PI 控制器
    pid_controller_t pll;

} smo_t;

void smo_init(smo_t *smo, float rs, float ls, float poles, float ts, float k_slide, float k_lpf, float boundary, float fc, float k_speed_lpf);

void smo_estimate(smo_t *smo);

float smo_get_bemf_alpha(smo_t *smo);

float smo_get_bemf_beta(smo_t *smo);

float smo_get_angle(smo_t *smo);

float smo_get_speed_rpm(smo_t *smo);

#endif /* smo.h */
