#ifndef __TIM_H__
#define __TIM_H__

#include "stm32g4xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;


/*
 * TIM1 PWM配置参数
 * TIM_CLK = 170 MHz / 定时器预分频因子
 * 系统时钟: 170MHz
 * PWM频率: 170MHz / (1+0) / (8500*2) = 10kHz（中心对齐模式下ARR计两次）
 * 死区时间: 约1us
 */
#define TIM1_PRESCALER 0  /* 预分频值 */
#define TIM1_PERIOD 8400  /* 自动重装载值（ARR） */
#define TIM1_DEADTIME 170 /* 死区时间：170/170MHz ≈ 1us */

void tim1_init(void);
void tim1_set_pwm_duty(float duty1, float duty2, float duty3);
float tim1_get_pwm_duty(uint32_t channel);

void tim3_init(void);

#endif /* __TIM_H__ */
