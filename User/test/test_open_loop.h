#ifndef __TEST_OPEN_LOOP_H__
#define __TEST_OPEN_LOOP_H__

#include "stm32g4xx_hal.h"
#include "./bsp/usart.h"
#include "./utils/vofa.h"
#include "./bsp/tim.h"
#include "./bsp/adc.h"
#include "./bsp/as5047.h"
#include "./foc/foc.h"
#include "./utils/fast_sin_cos.h"

/**
 * @brief FOC开环控制测试函数
 * 测试功能：
 * 1. 电机对齐测试
 * 2. 开环低速旋转测试
 * 3. 开环变速测试
 * 4. 开环正反转测试
 */
void test_open_loop(void);

/**
 * @brief 开环对齐测试
 * 执行电机对齐操作，确定编码器零点偏移
 */
void test_open_loop_alignment(void);

/**
 * @brief 开环低速旋转测试
 * 以固定低速开环控制电机旋转
 * @param voltage 电压幅值 (0.0-1.0)
 * @param speed_rpm 目标转速 (RPM)
 * @param duration_ms 测试持续时间 (毫秒)
 */
void test_open_loop_constant_speed(float voltage, float speed_rpm, uint32_t duration_ms);

/**
 * @brief 开环变速测试
 * 从低速到高速逐渐增加转速
 * @param voltage 电压幅值 (0.0-1.0)
 * @param speed_min_rpm 最小转速 (RPM)
 * @param speed_max_rpm 最大转速 (RPM)
 * @param step_rpm 转速步进 (RPM)
 * @param step_duration_ms 每步持续时间 (毫秒)
 */
void test_open_loop_variable_speed(float voltage, float speed_min_rpm, float speed_max_rpm, float step_rpm, uint32_t step_duration_ms);

/**
 * @brief 开环正反转测试
 * 测试电机正转和反转
 * @param voltage 电压幅值 (0.0-1.0)
 * @param speed_rpm 目标转速 (RPM)
 * @param cycles 正反转循环次数
 * @param duration_ms 每个方向持续时间 (毫秒)
 */
void test_open_loop_direction_test(float voltage, float speed_rpm, uint8_t cycles, uint32_t duration_ms);

#endif /* __TEST_OPEN_LOOP_H__ */