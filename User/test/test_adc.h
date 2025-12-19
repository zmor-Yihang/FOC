#ifndef __TEST_ADC_H__
#define __TEST_ADC_H__

#include "stm32g4xx_hal.h"

#include "./bsp/adc.h"
#include "./bsp/usart.h"
#include "./bsp/tim.h"
#include "./utils/vofa.h"

/**
 * @brief ADC测试函数
 * 测试ADC规则组DMA采样和注入组触发采样功能
 * 定期打印ADC采样结果，通过UART1输出到PC
 */
void test_adc(void);

#endif /* __TEST_ADC_H__ */
