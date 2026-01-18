#ifndef _TEST_SPEED_H__
#define _TEST_SPEED_H__

#include "stm32g4xx_hal.h"

/* 速度闭环测试 - 初始化并启动 */
void test_speed_loop_init(float target_rpm);

/* 打印状态信息 */
void test_speed_loop_print_status(void);

#endif 