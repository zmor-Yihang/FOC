#ifndef __KEY_H__
#define __KEY_H__

#include "stm32g4xx_hal.h"

/* 按键启停电机 */
void key_init(void);

/* 按键检测函数, 返回0表示按键未按下, 返回1表示按键按下 */
uint8_t key_scan(void);

#endif /* __KEY_H__ */
