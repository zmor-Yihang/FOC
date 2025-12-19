#ifndef __TEST_AS5047_H__
#define __TEST_AS5047_H__

#include "stm32g4xx_hal.h"
#include "./bsp/usart.h"
#include "./bsp/as5047.h"
#include "./utils/vofa.h"

/**
 * @brief AS5047P 编码器测试函数
 * @note  测试读取角度、诊断信息等
 */
void test_as5047(void);

#endif /* __TEST_AS5047_H__ */
