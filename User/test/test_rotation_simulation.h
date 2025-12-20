#ifndef __TEST_ROTATION_SIMULATION_H__
#define __TEST_ROTATION_SIMULATION_H__

#include "stm32g4xx_hal.h"
#include "./bsp/usart.h"
#include "./utils/vofa.h"
#include "./bsp/tim.h"
#include "./bsp/adc.h"
#include "./bsp/as5047.h"
#include "./foc/foc.h"
#include "./utils/fast_sin_cos.h"

/**
 * @brief 测试电角度自增模拟旋转功能
 */
void test_rotation_simulation(void);

#endif /* __TEST_ROTATION_SIMULATION_H__ */