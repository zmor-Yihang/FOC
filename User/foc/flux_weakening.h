#ifndef __FLUX_WEAKENING_H__
#define __FLUX_WEAKENING_H__

#include <math.h>
#include "pid.h"

typedef struct {
    float id_ref;           /* 输出的 Id 参考值 */
    float u_dc;             /* 母线电压 */
    float u_ref_ratio;      /* 弱磁起始电压占比 (如 0.95) */
    
    pid_controller_t pid;   /* 使用通用 PID 控制器 */

    float voltage_filter_const; /* 电压滤波系数 (0.0~1.0) */
    float u_current_filtered;   /* 滤波后的当前电压模值 */
} flux_weak_t;

/**
 * @brief 初始化弱磁控制器
 * @param flux_weak 句柄
 * @param u_dc 母线电压
 * @param u_ref_ratio 门槛比例 (0.90 ~ 0.95)
 * @param ki 积分系数
 * @param id_min 允许的最大负电流 (例如 -5.0)
 */
void flux_weak_init(flux_weak_t *flux_weak, float u_dc, float u_ref_ratio, float ki, float id_min);

/**
 * @brief 计算弱磁电流
 * @param flux_weak 句柄
 * @param v_d 当前 D 轴电压
 * @param v_q 当前 Q 轴电压
 * @return 目标 D 轴电流 (<= 0)
 */
float flux_weak_calculate(flux_weak_t *flux_weak, float v_d, float v_q);

#endif /* __FLUX_WEAKENING_H__ */
