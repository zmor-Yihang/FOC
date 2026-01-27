#ifndef __PRINT_H__
#define __PRINT_H__

#include <stdio.h>
#include "stm32g4xx_hal.h"

void printf_vofa(float *data, uint16_t num);

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
