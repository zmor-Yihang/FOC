#ifndef __TEST_PID_H__
#define __TEST_PID_H__

#include "foc/pid.h"
#include "utils/print.h"
#include "bsp/usart.h"
#include <stdio.h>

/* 测试FOC三闭环PID控制器（控制台输出） */
void test_foc_triple_loop(void);

/* 测试FOC三闭环并发送到VOFA+上位机 */
void test_foc_triple_loop_vofa(UART_HandleTypeDef *huart);

/* 测试单个PID控制器 */
void test_single_pid(void);

/* 模拟电机响应 */
float simulate_motor_response(float voltage, float load_torque, float dt);

#endif /* __TEST_PID_H__ */
