#include "main.h"

int main(void)
{
    HAL_Init();
    clock_init();
    usart1_init();

    led1_init(); // 初始化LED1
    led2_init(); // 初始化LED2，使能PWM输出

    key_init(); // 初始化按键

    as5047_init(); // 初始化AS5047P编码器

    adc1_init(); // 初始化ADC1

    tim3_init(); // 初始化TIM3用于编码器接口
    tim1_init(); // 初始化TIM1用于PWM输出


    test_foc_open(); // 测试FOC

    while (1)
    {

    }
}