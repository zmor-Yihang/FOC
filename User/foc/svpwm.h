#ifndef __SVPWM_H__
#define __SVPWM_H__

#include "clark_park.h"
#include "./utils/fast_sin_cos.h"

#define U_DC 12.0f /* 直流母线电压 (V) */

/**
 * @brief  SVPWM调制函数
 * @param  u_alphabeta - αβ轴电压 (V)
 * @return duty - 输出的三相占空比 (范围 0.0 ~ 1.0)
 */
abc_t svpwm_update(alphabeta_t u_alphabeta);

#endif /* __SVPWM_H__ */
