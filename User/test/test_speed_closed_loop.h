#ifndef __TEST_SPEED_CLOSED_LOOP_H__
#define __TEST_SPEED_CLOSED_LOOP_H__

#include "foc/foc.h"
#include "bsp/key.h"
#include "utils/delay.h"
#include "utils/print.h"

#include <stdio.h>

/* 速度闭环测试函数 */
void test_speed_closed_loop(void);

/* ADC注入组中断回调中调用的速度闭环处理函数 */
void speed_closed_loop_handler(void);

#endif /* __TEST_SPEED_CLOSED_LOOP_H__ */