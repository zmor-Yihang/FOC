#ifndef __LUENBERGER_H__
#define __LUENBERGER_H__

#include <math.h>
#include "utils/fast_sin_cos.h"
#include "pid.h"

// Luenberger 观测器结构体
typedef struct
{
    float i_alpha; // 实测电流 alpha
    float i_beta;  // 实测电流 beta
    float u_alpha; // 以此计算出的电压 alpha
    float u_beta;  // 以此计算出的电压 beta

    float rs;    // 定子电阻 (Ω)
    float ls;    // 定子电感 (H)
    float ts;    // 控制周期 (s)
    float poles; // 电机极对数

    float l1;       // 电流观测器增益
    float l2;       // 反电势观测器增益
    float k_pll_kp; // PLL KP
    float k_pll_ki; // PLL KI

    float i_alpha_est; // 估算电流 alpha (k)
    float i_beta_est;  // 估算电流 beta (k)
    float e_alpha_est; // 估算反电动势 alpha (k)
    float e_beta_est;  // 估算反电动势 beta (k)

    float theta_est;   // 估算角度 (rad)
    float speed_est;   // 估算速度 (rpm)
    float speed_est_filt; // 滤波后的速度 (rpm)
    float k_speed_lpf; // 速度低通滤波系数
    float speed_rad_s; // 估算电角速度 (rad/s)

    // PLL PI 控制器
    pid_controller_t pll;

} luenberger_t;

/**
 * @brief 初始化 Luenberger 观测器
 * @param luenberger 观测器句柄
 * @param rs 定子电阻
 * @param ls 定子电感
 * @param poles 极对数
 * @param ts 采样周期
 * @param l1 电流增益
 * @param l2 反电势增益
 * @param pll_fc PLL 截止频率 (Hz)
 * @param k_speed_lpf 速度滤波系数
 */
void luenberger_init(luenberger_t *luenberger, float rs, float ls, float poles, float ts, float l1, float l2, float pll_fc, float k_speed_lpf);

/**
 * @brief 运行 Luenberger 观测器
 * @param luenberger 观测器句柄
 */
void luenberger_estimate(luenberger_t *luenberger);

/**
 * @brief 获取估算的角度 (rad)
 * @param luenberger 观测器句柄
 * @return float 角度
 */
float luenberger_get_angle(luenberger_t *luenberger);

/**
 * @brief 获取估算的速度 (rpm)
 * @param luenberger 观测器句柄
 * @return float 速度
 */
float luenberger_get_speed_rpm(luenberger_t *luenberger);

#endif /* __LUENBERGER_H__ */
