#ifndef __TEST_CURRENT_CLOSED_LOOP_H__
#define __TEST_CURRENT_CLOSED_LOOP_H__

#include "foc/foc.h"
#include "bsp/key.h"
#include "utils/delay.h"
#include <stdio.h>

/* 电流双闭环测试函数 */
void test_current_closed_loop(void);

/* ADC注入组中断回调中调用的电流闭环处理函数 */
void current_closed_loop_handler(void);

#endif /* __TEST_CURRENT_CLOSED_LOOP_H__ */
