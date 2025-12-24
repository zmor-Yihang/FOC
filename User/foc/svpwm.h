#ifndef __SVPWM_H__
#define __SVPWM_H__

#include "./config/type_config.h"
#include "./utils/fast_sin_cos.h"

/**
 * @brief  SVPWM调制函数
 * @param  u_alphabeta - αβ轴电压 (V)
 * @return duty - 输出的三相占空比 (范围 0.0 ~ 1.0)
 */
abc_t svpwm_update(alphabeta_t u_alphabeta);

#endif /* __SVPWM_H__ */
