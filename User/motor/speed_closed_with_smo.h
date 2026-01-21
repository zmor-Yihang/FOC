#ifndef __SPEED_CLOSED_WITH_SMO_H__
#define __SPEED_CLOSED_WITH_SMO_H__

#include <stdio.h>
#include "foc/foc.h"
#include "foc/smo.h"
#include "bsp/as5047.h"
#include "utils/print.h"

/**
 * @brief 初始化有感速度闭环控制（同时运行SMO观测器）
 * @param speed_rpm 目标速度 (RPM)
 * @note 使用编码器反馈进行控制，SMO仅用于观测对比
 */
void speed_closed_with_smo_init(float speed_rpm);

/**
 * @brief 打印编码器和SMO的速度、角度信息
 * @note 输出格式：编码器速度, 编码器角度, SMO速度, SMO角度
 */
void print_speed_smo_info(void);

#endif /* __SPEED_CLOSED_WITH_SMO_H__ */
