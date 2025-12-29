#ifndef __PRINT_H__
#define __PRINT_H__

#include <stdio.h>   
#include <stdarg.h>  // 用于处理可变参数
#include "stm32g4xx_hal.h" 

#define printf_period(period_ms, fmt, ...)                        \
    do                                                            \
    {                                                             \
        static uint32_t _last_tick_##__LINE__ = 0;                \
        if (HAL_GetTick() - _last_tick_##__LINE__ >= (period_ms)) \
        {                                                         \
            _last_tick_##__LINE__ = HAL_GetTick();                \
            printf(fmt, ##__VA_ARGS__);                           \
        }                                                         \
    } while (0)

#endif /* __PRINT_H__ */

