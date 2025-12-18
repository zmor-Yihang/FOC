#ifndef __SVPWM_H__
#define __SVPWM_H__

#include "./config/type_config.h"
#include "./utils/fast_sin_cos.h"

/**
 * @brief  SVPWM调制函数
 * @param  u_abc - 输入的三相调制电压 (归一化, 范围 -1.0 ~ 1.0)
 * @return duty - 输出的三相占空比 (范围 0.0 ~ 1.0)
 */
abc_t svpwm_update(abc_t u_abc);

#endif /* __SVPWM_H__ */
