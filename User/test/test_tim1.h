#ifndef __TEST_TIM1_H__
#define __TEST_TIM1_H__

#include "stm32g4xx_hal.h"
#include "./bsp/usart.h"
#include "./utils/print.h"
#include "./bsp/tim.h"
#include "./utils/fast_sin_cos.h"

/**
 * @brief TIM1 PWM发波测试函数
 * 测试TIM1三相PWM输出和互补通道功能
 * 动态改变三个通道的占空比，输出SVPWM波形
 */
void test_tim1(void);

#endif /* __TEST_TIM1_H__ */
