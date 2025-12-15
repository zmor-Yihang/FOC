#ifndef __VOFA_H__
#define __VOFA_H_

#include <stdio.h>   // 用于 sprintf, vsnprintf
#include <stdarg.h>  // 用于处理可变参数
#include <string.h>  // 用于 strlen

#include "stm32g4xx_hal.h" 
#include "bsp/usart1.h" 

void vofa_print(UART_HandleTypeDef *huart, const char *format, ...);

#endif /* __VOFA_H__ */

