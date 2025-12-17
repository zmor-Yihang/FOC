#ifndef __LED_H__
#define __LED_H__

#include "stm32g4xx_hal.h"

/* LED1 (PB12)，红灯 */
void led1_init(void);
void led1_on(void);
void led1_off(void);
void led1_toggle(void);

/* LED2 (PA11)，绿灯 */
void led2_init(void);
void led2_on(void);
void led2_off(void);
void led2_toggle(void);

/* LED3 (PB11)，电机错误指示灯 */
void led3_init(void);

#endif /* __LED_H__ */
